# DECODE Robot Code — Architecture Outline

> Comprehensive reference for the FTC DECODE shoot-assist TeleOp. All tuning knobs are
> `public static` in `RobotHardware` so they appear as Panels Configurables and can be
> adjusted live without redeploying.

---

## Table of Contents

1. [Overview](#1-overview)
2. [Hardware Layout](#2-hardware-layout)
3. [Drive Subsystem](#3-drive-subsystem) — field-centric mecanum with voltage + torque control
4. [Flywheel Subsystem](#4-flywheel-subsystem) — closed-loop velocity with feedforward
5. [Hood Subsystem](#5-hood-subsystem) — ServoExGroup, LUT aiming, kH compensation
6. [Intake Subsystem](#6-intake-subsystem)
7. [Gate Subsystem](#7-gate-subsystem)
8. [State Machine — MasterController](#8-state-machine--mastercontroller)
9. [Shoot Sequence — ShootCommand](#9-shoot-sequence--shootcommand)
10. [Launch Zone RTP — LaunchZoneRTPCommand](#10-launch-zone-rtp--launchzonertpcommand)
11. [Limelight Integration](#11-limelight-integration)
12. [Localization — Pedro Pathing + Kalman Filter](#12-localization--pedro-pathing--kalman-filter)
13. [Driver Controls (Gamepads)](#13-driver-controls-gamepads)
14. [Telemetry — Panels](#14-telemetry--panels)
15. [Autonomous — MainAuto](#15-autonomous--mainauto)

---

## 1. Overview

The robot runs a **shoot-assist TeleOp loop** for the DECODE game. The driver's job is to drive and intake; the code handles alignment, aiming, and shooting.

```
Gamepad1                          Robot
  left stick ──────────────────► Drive   (field-centric mecanum)
  right stick ──────────────────► Drive   (rotation)
  right trigger held ────────────► MasterController → ALIGNING / ALIGNED
  left trigger (ALIGNED + ready) → MasterController → SHOOT
  right bumper ─────────────────► MasterController → INTAKE
  left bumper ──────────────────► MasterController → INTAKE_REVERSE

Gamepad2
  dpad up/down ─────────────────► FlywheelSubsystem.addOffset()
  dpad left/right ─────────────► RobotHardware.hoodAngleOffset
  share ────────────────────────► override isReadyToShoot = true
  share + option ───────────────► Launch Zone RTP active
  circle ───────────────────────► MasterController → INTAKE (instant)
```

**Libraries used:**
- **SolversLib** — motor/servo wrappers, gamepad helpers, command base classes, telemetry
- **Pedro Pathing** — deadwheel odometry, pose tracking, follower (auto)
- **Marrow** (optional) — pose persistence across INIT loops via Settings
- **Panels** — live tuning via Configurables, field view, graphs
- **Limelight API** (FTC SDK) — AprilTag/MegaTag 2 vision

---

## 2. Hardware Layout

All hardware is declared and initialised in `RobotHardware.init()`.

### Drive Motors

```
FL  MotorEx(GoBILDA.RPM_435)  setZeroPowerBehavior(BRAKE)   normal direction
FR  MotorEx(GoBILDA.RPM_435)  setZeroPowerBehavior(BRAKE)   setInverted(true)
BL  MotorEx(GoBILDA.RPM_435)  setZeroPowerBehavior(BRAKE)   normal direction
BR  MotorEx(GoBILDA.RPM_435)  setZeroPowerBehavior(BRAKE)   setInverted(true)
```

- `GoBILDA.RPM_435` sets `ACHIEVABLE_MAX_TICKS_PER_SECOND = 2769` (384.5 CPR × 435 RPM / 60 s)
- **BRAKE** mode means motors actively resist motion when sticks are centered — firm stops
- Right side motors (`fr`, `br`) are software-inverted so that positive power = forward on both sides

### Flywheel Motors

```
flywheelL  MotorEx(RUN_MODE = VelocityControl)  setFeedforwardCoefficients(kS=0.15, kV=12/maxVel)
flywheelR  MotorEx(RUN_MODE = VelocityControl)  setFeedforwardCoefficients(kS=0.15, kV=12/maxVel)
```

- `MotorEx` in `VelocityControl` — per-cycle voltage loop with PIDF + torque clamping
- Feedforward: `kV = 12 / maxVelocity` (set(1.0) = full speed), `kS = 0.15 V` (static friction)
- Motor physics constants (GoBilda 435 RPM, 13.7:1 planetary):

| Constant | Value | How derived |
|---|---|---|
| `FLYWHEEL_TPR` | 384.5 ticks/rev | encoder CPR |
| `FLYWHEEL_OMEGA` | 45.55 rad/s | 435 RPM × 2π/60 |
| `FLYWHEEL_R` | 1.30 Ω | 12 V / 9.2 A stall |
| `FLYWHEEL_V_RES` | 0.326 V | 0.25 A × R (no-load current) |
| `FLYWHEEL_K_EMF` | 0.256 V/(rad/s) | (12 − V_RES) / OMEGA |

### Intake Motor

```
intake  Motor  (no encoder — raw power)
```

- **No encoder** — velocity feedback is unavailable, so back-EMF is estimated from the *commanded* speed rather than measured
- Pure feedforward voltage control: `vFF = K_EMF × ω_cmd + kS × sign(command)`
- Battery-compensated and current-clamped, but no closed-loop PID
- Motor physics constants (GoBilda 312 RPM, 19.2:1 planetary):

| Constant | Value | How derived |
|---|---|---|
| `INTAKE_TPR` | 384.5 ticks/rev | encoder CPR |
| `INTAKE_OMEGA` | 32.63 rad/s | 312 RPM × 2π/60 |
| `INTAKE_R` | 1.30 Ω | 12 V / 9.2 A stall |
| `INTAKE_V_RES` | 0.326 V | 0.25 A × R |
| `INTAKE_K_EMF` | 0.357 V/(rad/s) | (12 − V_RES) / OMEGA |

### Hood Servos (ServoExGroup)

```
hoodL  ServoEx("hoodL")          leader
hoodR  ServoEx("hoodR")  setInverted(true)   follower
hood   ServoExGroup(hoodL, hoodR)   ← single handle used by HoodSubsystem
```

- `hoodR` is **hardware-reversed** (`setInverted(true)` → SDK `Servo.Direction.REVERSE`)
  so sending the same position value makes both servos move in opposite physical directions
- `ServoExGroup` broadcasts the leader's position to all followers; caching is inherited from `ServoEx`

### Gate Servo

```
gate  ServoEx
  GATE_CLOSE_POSITION = 0.0   (closed at init)
  GATE_OPEN_POSITION  = 0.55
```

### IMU

```
IMU  RevHubOrientationOnRobot.LogoFacingDirection.UP
                 UsbFacingDirection.LEFT
```

- `getYawRadians()` returns the robot's heading relative to the field
- Resettable at runtime via `imu.resetYaw()` (Options button)

### Voltage Sensor

```
voltageSensor = hwMap.voltageSensor.get("Control Hub")
```

- Read by `RobotHardware.batteryVoltage()` every loop
- Used to scale all motor voltages so performance is consistent from 12 V down to ~10 V

### Lynx Bulk Caching

```java
controlHub = (parent LynxModule)
controlHub.setBulkCachingMode(MANUAL)
```

`hw.clearBulkCache()` is called once per loop to minimise USB round-trips.

---

## 3. Drive Subsystem

**File:** `subsystems/DriveSubsystem.java`
**Parent:** `SolversLib MecanumDrive`

`DriveSubsystem` extends `MecanumDrive` to inherit field-centric kinematics, but **fully overrides `driveFieldCentric()`** to apply voltage-based torque control before sending power to the motors.

### Control Chain

```
gamepad sticks
    │
    ▼
MecanumDrive.driveFieldCentric()
    │  (field-centric rotation, inverse kinematics → 4 wheel fractions)
    ▼
applyVoltageControl(fl, fr, bl, br)   ← overridden in DriveSubsystem
    │
    ├─ vTarget  = frac × 12 V
    ├─ vBackEmf = K_EMF × ω            (opposes applied voltage)
    ├─ iTarget  = |frac| < 0.01 → 0 (coast near zero)
    │             else → MAX_CURRENT   (3 A — torque clamp)
    ├─ vMin = vBackEmf − iTarget × R_MOTOR
    ├─ vMax = vBackEmf + iTarget × R_MOTOR
    ├─ vClamped = clamp(vTarget, vMin, vMax)
    └─ power = vClamped / batteryVoltage
    │
    ▼
motor.set(power)    (via MotorEx)
```

### Motor Constants (GoBilda 435 RPM)

| Constant | Value | How derived |
|---|---|---|
| `TPR` | 384.5 ticks/rev | encoder CPR |
| `I_STALL` | 9.2 A | motor spec |
| `R_MOTOR` | 1.30 Ω | 12 V / 9.2 A |
| `V_RESISTOR` | 0.326 V | 0.25 A no-load × R |
| `OMEGA_NOLOAD` | 45.55 rad/s | 435 RPM × 2π/60 |
| `K_EMF` | 0.256 V/(rad/s) | (12 − 0.326) / 45.55 |
| `MAX_CURRENT` | **3.0 A** (tunable) | current clamp threshold |
| `STALL_CHECK_INTERVAL` | **10 loops** (tunable) | debounce for `isStalling()` |

### Feedforward Setup

```java
double maxVel = 384.5 * 435.0 / 60.0;  // ≈ 2769 ticks/s
double kV = 12.0 / maxVel;              // V·s/tick  → set(1.0) = maxVel
double kS = 0.15;                       // V static friction
motor.setFeedforwardCoefficients(kS, kV);
```

### Stalling Detection

`isStalling()` returns `true` if any motor has exceeded `MAX_CURRENT` within the last `STALL_CHECK_INTERVAL` loops. Available to inhibit auto-align or trigger an LED.

### Zero Power Behavior

All four drive motors are **BRAKE** — when `power = 0` they resist motion rather than coasting. Combined with voltage control, this gives firm, consistent stops.

---

## 4. Flywheel Subsystem

**File:** `subsystems/FlywheelSubsystem.java`

Two motors in `VelocityControl` mode, updated every loop with a **manual PIDF + back-EMF compensation + torque limiting** loop. All motor physics constants live in `RobotHardware`.

### Voltage Loop (per motor, per cycle)

```
ω_measured = motor.getVelocity() / TPR × 2π          (rad/s)
vBackEmf   = K_EMF × ω_measured                      (opposes applied voltage)

PID corrective voltage:
  err        = targetVelocity − measuredVelocity       (ticks/s)
  integral   = clamp(integral + err·dt, −12, 12)     (anti-windup)
  derivative = (err − prevErr) / dt
  vPID      = kP·err + kI·integral + kD·derivative + kF·targetVelocity

vTarget = vPID + vBackEmf                            (back-EMF compensation)

Torque limiting — always active (flywheels are load-bearing):
  iTarget  = FLYWHEEL_MAX_CURRENT                    (3 A — tunable)
  vMin     = vBackEmf − iTarget × R_MOTOR
  vMax     = vBackEmf + iTarget × R_MOTOR
  vClamped = clamp(vTarget, vMin, vMax)

power = vClamped / batteryVoltage
```

> Back-EMF compensation means `vPID` is a correction around the motor's actual operating point rather than around zero, so the PID responds correctly at all speeds.

### Tunable Constants

| Constant | Default | Meaning |
|---|---|---|
| `FLYWHEEL_TARGET_VELOCITY` | 2800 ticks/s | speed both motors try to reach |
| `FLYWHEEL_READY_TOLERANCE` | 150 ticks/s | `isReadyToShoot` velocity threshold |
| `FLYWHEEL_L_KP/KI/KD/KF` | 0.0001 / 0.001 / 0 / 0 | left motor PID |
| `FLYWHEEL_R_KP/KI/KD/KF` | 0.0001 / 0.001 / 0 / 0 | right motor PID |
| `FLYWHEEL_MAX_CURRENT` | 3.0 A | torque clamp — protects motors under load |
| `flywheelVelocityOffset` | 0 (live) | gamepad2 dpad up/down delta ±50 |

- Left and right have separate PID gains to handle slight mechanical differences
- `addOffset(delta)` adjusts `targetVelocity` live; `reset()` zeroes integral and derivative history

---

## 5. Hood Subsystem

**File:** `subsystems/HoodSubsystem.java`

### Single-Value Control via ServoExGroup

```
HoodSubsystem
    │
    ▼
ServoExGroup
    ├─ hoodL (leader)  setPosition(value)
    └─ hoodR (follower)  receives leader's position × hardware inversion → same physical angle
```

Both servos physically mirror each other (one reversed) so a single `setPosition(p)` call moves the entire hood.

### Hardstop Enforcement

`setRange(0, 1)` on the underlying `ServoEx` objects sets the PWM scale but does **not** clamp. `clamp(position, HOOD_MIN_POSITION, HOOD_MAX_POSITION)` is applied before every write.

| Constant | Value |
|---|---|
| `HOOD_MIN_POSITION` | 0.05 |
| `HOOD_MAX_POSITION` | 0.80 |
| `HOOD_ANGLE_OFFSET_JUMP` | 0.02 (dpad left/right increment) |

### Hood LUT (Aiming)

`RobotHardware.HOOD_LUT` is a linear-interpolation lookup table from measured distance (inches) to servo position. It is built from `HOOD_DISTANCE_SAMPLES` and `HOOD_POSITION_SAMPLES`.

```java
hoodPosition(distance, offset) → clamp(LUT.get(distance) + offset, MIN, MAX)
```

### kH Compensation

When a note is loaded the flywheel decelerates slightly. `kH` compensation adds extra hood angle proportional to the velocity drop:

```
hoodOffset = (targetVelocity − actualVelocity) × HOOD_COMPENSATION_COEFFICIENT
finalPos   = basePosition + hoodOffset
```

During `SHOOT` state the base position is **locked** at the moment the shot starts; only the kH compensation delta is updated while the note is in flight.

---

## 6. Intake Subsystem

**File:** `subsystems/IntakeSubsystem.java`

`Motor` (no encoder). The intake drives at ±100% of `INTAKE_MAX_VELOCITY`. Because no encoder is present, back-EMF is estimated from the **commanded** speed (pure feedforward) rather than measured. No closed-loop PID is used.

### Voltage Loop (per cycle — pure feedforward, no velocity feedback)

```
ω_command = command × INTAKE_MAX_VELOCITY / INTAKE_TPR × 2π    (rad/s)
vFF       = INTAKE_K_EMF × ω_command                            (velocity feedforward)
vS        = 0.15 × sign(command)                               (static friction kS)
vBackEmf  = vFF + vS                                            (estimated opposing EMF)

iTarget = |command| < 0.01 → 0  (coast when stopped)
          otherwise → INTAKE_MAX_CURRENT               (4 A — protects motor on jam)

vMin     = vBackEmf − iTarget × INTAKE_R
vMax     = vBackEmf + iTarget × INTAKE_R
vClamped = clamp(command × 12 V, vMin, vMax)
power    = vClamped / batteryVoltage
```

When stopped (`|command| < 0.01`) the motor coasts — no current is applied and no resistance is generated. When running, current is clamped to `INTAKE_MAX_CURRENT` (4 A) so the motor is protected if a note jams the intake.

### Commands

| Method | Effect |
|---|---|
| `runForward()` | Drive at +100% of `INTAKE_MAX_VELOCITY` |
| `runReverse()` | Drive at −100% of `INTAKE_MAX_VELOCITY` |
| `stop()` | Cut power immediately (via `stopMotor()`) |
| `isRunning()` | `true` if `\|motor.get()\| > 0.01` |

### Tunable Constants

| Constant | Default | Meaning |
|---|---|---|
| `INTAKE_MAX_CURRENT` | 4.0 A | current clamp — protects motor under jam |
| `INTAKE_MAX_VELOCITY` | ~2005 ticks/s | derived: TPR × RPM / 60 |

---

## 7. Gate Subsystem

**File:** `subsystems/GateSubsystem.java`

Single `ServoEx`. Opens and closes the note gate.

| Position | Value | Use |
|---|---|---|
| `GATE_CLOSE_POSITION` | 0.0 | Default at init; closed during intake and alignment |
| `GATE_OPEN_POSITION` | 0.55 | Opened during `ShootCommand` |

`isOpen()` returns true if `gate.getPosition() > (OPEN + CLOSE) / 2`.

---

## 8. State Machine — MasterController

**File:** `MasterController.java`
**States:** `RobotState` enum

```
 INIT
  │
  ▼ press play
 INTAKE  ◄──────────────────────┐
  │                               │
  │ left bumper ─────────────┐   │
  │                           │   │
  │ ▼                        │   │
INTAKE_REVERSE              INTAKE
  │                           │
  │ release bumper ───────────┘   │
  │                               │
  │ right trigger held ─────┐     │
  │                         │     │
  │ ▼                      │     │
ALIGNING                   │     │
  │  (accumulate timer      │     │
  │   while tx < 2°)        │     │
  │  + 0.15 s ─────────┐   │     │
  │                   │   │     │
  │ ▼                │   │     │
ALIGNED  ──── target lost ──┘   │
  │                               │
  │ left trigger (edge) + ───────►│
  │ isReadyToShoot OR share ──► SHOOT
  │   (non-blocking timer)
  │              │
  │              ▼
  │           INTAKE  (after SHOOT_DELAY)
```

### Priority Rules

1. **Left bumper is king** — checked first; overrides everything and drives `INTAKE_REVERSE`
2. **Right bumper** → `INTAKE` (stop reverse / stop intake)
3. **Circle (gamepad2)** → instant `INTAKE`
4. **During SHOOT** — gamepads are fully neglected until the timer expires

### Alignment Timer

The `ALIGNMENT_DELAY` (0.15 s) accumulates **only while the limelight crosshair is within 2° of the target**. If the target is lost the timer resets, so the driver must hold steady alignment to proceed.

---

## 9. Shoot Sequence — ShootCommand

**File:** `commands/ShootCommand.java`
**Non-blocking** — uses `ElapsedTime`, no `Thread.sleep`.

```
execute() called every loop while shooting:
  ├─ t = 0  → gate.open(), intake.runForward()
  ├─ t ≥ SHOOT_DELAY (2.5 s) → gate.close(), intake.stop(), onComplete.run()
```

`isFinished()` returns `true` after the delay. `end()` is called to clean up if interrupted.

---

## 10. Launch Zone RTP — LaunchZoneRTPCommand

**File:** `commands/LaunchZoneRTPCommand.java`

**Active when:** gamepad1 right trigger held AND state = `ALIGNING` or `ALIGNED`.

### Launch Zones

Two Marrow `PolygonZone` instances define the pull regions:

```
CLOSE_ZONE vertices:  (144, 144) — (72, 72) — (0, 144)
FAR_ZONE  vertices:    (48, 0)   — (72, 24)  — (96, 0)
```

`CLOSE_ZONE` is the large right-triangle in the scoring-corner area; `FAR_ZONE` is the smaller triangle near the field center. Both zones use the Marrow `Zones` API from `com.skeletonarmyftc.marrow`.

### Algorithm

1. **Zone selection:** compute `CLOSE_ZONE.distanceTo(robot)` and `FAR_ZONE.distanceTo(robot)`; pick the nearer zone
2. **Closest-point-on-polygon:** iterate every edge of the selected zone; for each edge find the closest point on that edge to the robot; return the minimum across all edges
3. **Pull vector:** compute the unit vector from robot → closest boundary point
4. **World-frame driver input:** rotate joystick by `robotH`
5. **Blend:** `blended = pull × (1 − WEIGHT) + driverWorld × WEIGHT`; `VECTOR_WEIGHT_DRIVER = 0.6`
6. **Normalize** to unit magnitude
7. If robot is **not active** (trigger released or wrong state): pass raw driver input through unchanged

The blended forward/strafe is written to `MainTeleOp.blendedFwd / blendedStrafe` and fed into `drive.driveFieldCentric()`.

### Tuning

| Constant | Default | Effect |
|---|---|---|
| `VECTOR_WEIGHT_DRIVER` | 0.6 | Driver authority — see §10 table in `tuning.md` |

---

## 11. Distance Measurement (Pose-Based)

### How Distance Is Computed

```
dx = GOAL_COORDS.x − robotX
dy = GOAL_COORDS.y − robotY
distance = √(dx² + dy²)
```

`robotX / robotY / robotH` are the filtered Kalman pose (Pedro `Follower.getPose()` after fusing odometry + MegaTag 2 vision). `GOAL_COORDS` is switched per-alliance in `init_loop()` from `RED_GOAL_COORDS` / `BLUE_GOAL_COORDS` in `RobotHardware`.

Rejected if `distance < LIMELIGHT_DIST_MIN (5 in)` or `> LIMELIGHT_DIST_MAX (120 in)`.

### Uses

- **Hood aiming** — `hoodPosition(distance)` → interpolates from `HOOD_DISTANCE_SAMPLES` → `HOOD_POSITION_SAMPLES` lookup table
- **`isReadyToShoot`** — requires a valid pose distance to allow shooting
- **Telemetry** — displayed as `Pose dist (in)` in Panels

### Crosshair Servoing (Auto-Align)

```
tx = Limelight TX (degrees off-centre)

rotationCorrection = tx × 0.05   (P controller on crosshair error)
```

If Limelight has no valid target, falls back to **odometry heading** toward `GOAL_COORDS`:

```
headingToGoal = atan2(GOAL.y − robotY, GOAL.x − robotX)
headingError  = headingToGoal − robotH
rotationCorrection = headingError × 0.05
```

### MegaTag 2 Pose Update

Every loop a fresh `botpose_MT2` reading is used to correct the Kalman drift filter (see §12).

### Pipelines

| Pipeline | Index | Alliance |
|---|---|---|
| RED | 0 | RED |
| BLUE | 1 | BLUE |

Switched during `init_loop()` alliance selection and locked at `start()`.

---

## 12. Localization — Pedro Pathing + Kalman Filter

### Pedro Pathing

```
follower = Constants.createFollower(hardwareMap)
follower.startTeleopDrive()
follower.update()   // every loop
follower.getPose() → Pose(x, y, heading)
```

Deadwheel odometry gives continuous pose between vision updates.

### Kalman Drift Filter (`DriftFilter` class in RobotHardware)

Three independent 1-D Kalman filters, one per axis (X, Y, Heading).

**Predict step** (every loop, with timestep `dt`):
```
P += qBias² × dt      // covariance grows (random walk / gyro drift)
```

**Update step** (when fresh MegaTag 2 reading is available):
```
measuredDrift = odomReading − visionReading
K = P / (P + rVision)          // Kalman gain
drift = drift + K × (measuredDrift − drift)
P = (1 − K) × P
```

**Corrected pose:**
```
correctedReading = odomReading − drift
```

| Tunable | Default | Meaning |
|---|---|---|
| `KALMAN_Q_BIAS_X/Y/H` | 0.02 / 0.02 / 0.005 | process noise (sqrt of variance per √s) |
| `KALMAN_R_VISION_X/Y/H` | 4.0 / 4.0 / 0.01 | measurement noise variance |

### Pose Getters Used Across the Code

```
getCorrectedX(odomX) → Kalman-corrected X
getCorrectedY(odomY) → Kalman-corrected Y
getCorrectedH(odomH) → Kalman-corrected heading
```

Used by:
- Launch Zone RTP (robot position)
- Odometry heading fallback for auto-align
- Telemetry display

---

## 13. Driver Controls (Gamepads)

All raw reads go through **`GamepadEx`** from SolversLib for edge detection.

### Gamepad 1 (Driver)

| Input | Action |
|---|---|
| Left stick X/Y | Forward/strafe drive |
| Right stick X | Rotation |
| Right trigger (`> 0.5`) | Hold → `ALIGNING` |
| Left trigger (edge) | Fire shot (requires `ALIGNED` + `isReadyToShoot`) |
| Right bumper | `INTAKE` |
| Left bumper | `INTAKE_REVERSE` (king — overrides everything) |
| Triangle | Select RED alliance (in `init_loop`) |
| Circle | Select BLUE alliance (in `init_loop`) |
| Options | Reset IMU yaw |
| Touchpad | Reset deadwheel pose to `(0, 0, 0)` + reinit Kalman |

### Gamepad 2 (Operator)

| Input | Action |
|---|---|
| Dpad up | Increase flywheel velocity offset (+50 ticks/s) |
| Dpad down | Decrease flywheel velocity offset (−50 ticks/s) |
| Dpad left | Increase hood angle offset (+0.02) |
| Dpad right | Decrease hood angle offset (−0.02) |
| Share held | Override `isReadyToShoot = true` |
| Share + Options | Enable Launch Zone RTP |
| Circle | Instant `INTAKE` (disregards all other state) |

### isReadyToShoot

All three conditions must be true:
1. State is `ALIGNED`
2. Limelight has a valid target
3. Both flywheel velocities within `FLYWHEEL_READY_TOLERANCE` of target

When `isReadyToShoot` changes, both gamepads receive **two haptic blips** to alert the driver.

---

## 14. Telemetry — Panels

`RobotHardware.panels` holds the `Panels` instance. `RobotHardware.telemetryData` wraps `panels.getTelemetry()` and is the primary telemetry bus.

Displayed every loop:
- Alliance, State
- Flywheel velocity (L, R, target)
- Gate position, Hood position
- `isReadyToShoot`, Share override
- Limelight distance, `tx`, `ty`
- Pose (X, Y, heading in degrees)
- Flywheel velocity offset, Hood angle offset

All constants are `public static` in `RobotHardware` and annotated with `@Configurable`, so they appear as live-tuning sliders in the Panels dashboard.

---

## 15. Autonomous — MainAuto

**File:** `MainAuto.java`

Autonomous uses **Pedro Pathing** for all drive movement — the `Follower` class handles path following. The `DriveSubsystem` is **not used** in auto (Pedro handles it internally via its own mecanum kinematics).

### Subsystems Initialised

| Subsystem | Purpose |
|---|---|
| `FlywheelSubsystem` | Spin up flywheels during auto |
| `IntakeSubsystem` | Score preload, collect from wall |
| `GateSubsystem` | Control note flow |
| `HoodSubsystem` | Aim hood per path position |

### Pedro Pathing Setup

Pedro Pathing uses a **Two-Wheel Localizer** (forward pod + lateral pod + IMU) and a **Mecanum drivetrain**, both fully configured in `Constants.java`. All values are Panels-tunable via `Configurable` annotations.

```
Constants.createFollower(hardwareMap)
  └─ TwoWheelLocalizer    ← forward pod (FL motor) + lateral pod (BR motor) + IMU
  └─ MecanumDrivetrain    ← FL / BL / FR / BR motors
  └─ FollowerConstants    ← mass, PIDF, velocity, ZPA, centripetal
  └─ PathConstraints      ← maxVel, maxAccel, maxJerk, maxAngVel
```

**Per-OpMode initialization:**
```java
// In init() or init_loop()
follower = Constants.createFollower(hardwareMap);
hw.initLocalizer();   // Kalman drift filters (x/y/h)
hw.clearBulkCache();

// In start()
follower.setStartingPose(new Pose(startX, startY, startH));

// In loop()
follower.update();     // every loop — updates pose from odometry + IMU
```

**Tuning order:**
1. Forward Tuner → `FORWARD_TICKS_TO_INCHES`
2. Lateral Tuner → `STRAFE_TICKS_TO_INCHES`
3. Offsets Tuner → `FORWARD_POD_Y`, `STRAFE_POD_X`
4. Forward Velocity Tuner → `X_VELOCITY`
5. Lateral Velocity Tuner → `Y_VELOCITY`
6. Forward / Lateral ZPA Tuners → `FORWARD_ZPA`, `LATERAL_ZPA`
7. Heading Tuner → `HEADING_P / I / D / F`
8. Translational Tuner → `TRANSLATIONAL_P / I / D / F`
9. Drive Tuner → `DRIVE_P / I / D / F / T` + `BRAKING_STRENGTH`
10. Centripetal Tuner → `CENTRIPETAL_SCALING`

**IMU orientation** — set `IMU_LOGO_FACING` and `IMU_USB_FACING` in `Constants.java` to match physical Control Hub mounting (default: UP + LEFT).

### Limelight in Auto

MegaTag 2 (Limelight pose estimation) runs continuously during auto, updating the Kalman filter so the `Follower`'s pose stays accurate even without deadwheel odometry.

---

## Constants Quick Reference

| Constant | Value | Location |
|---|---|---|
| `FLYWHEEL_TARGET_VELOCITY` | 2800 ticks/s | `RobotHardware` |
| `FLYWHEEL_READY_TOLERANCE` | 150 ticks/s | `RobotHardware` |
| `FLYWHEEL_MAX_CURRENT` | 3.0 A | `RobotHardware` |
| `FLYWHEEL_TPR / RPM` | 384.5 / 435 | `RobotHardware` |
| `FLYWHEEL_K_EMF` | 0.256 V/(rad/s) | `RobotHardware` |
| `FLYWHEEL_R` | 1.30 Ω | `RobotHardware` |
| `FLYWHEEL_L_KP/KI/KD/KF` | 0.0001 / 0.001 / 0 / 0 | `RobotHardware` |
| `FLYWHEEL_R_KP/KI/KD/KF` | 0.0001 / 0.001 / 0 / 0 | `RobotHardware` |
| `INTAKE_MAX_CURRENT` | 4.0 A | `RobotHardware` |
| `INTAKE_MAX_VELOCITY` | ~2005 ticks/s | `RobotHardware` |
| `INTAKE_TPR / RPM` | 384.5 / 312 | `RobotHardware` |
| `INTAKE_K_EMF` | 0.357 V/(rad/s) | `RobotHardware` |
| `INTAKE_R` | 1.30 Ω | `RobotHardware` |
| `SHOOT_DELAY` | 2.5 s | `RobotHardware` |
| `GATE_OPEN_POSITION` | 0.55 | `RobotHardware` |
| `GATE_CLOSE_POSITION` | 0.0 | `RobotHardware` |
| `HOOD_MIN/MAX_POSITION` | 0.05 / 0.80 | `RobotHardware` |
| `HOOD_COMPENSATION_COEFFICIENT` | 0.0005 | `RobotHardware` |
| `DRIVE_MAX_CURRENT` | 3.0 A | `DriveSubsystem` |
| `DRIVE_STALL_CHECK_INTERVAL` | 10 loops | `DriveSubsystem` |
| `ALIGNMENT_DELAY` | 0.15 s | `RobotHardware` |
| `VECTOR_WEIGHT_DRIVER` | 0.6 | `RobotHardware` |
| `LIMELIGHT_MOUNT_ANGLE` | 25° | `RobotHardware` |
| `RED_GOAL_COORDS` | (144, 144) — red alliance target | `RobotHardware` |
| `BLUE_GOAL_COORDS` | (0, 144) — blue alliance target | `RobotHardware` |
| `CLOSE_LAUNCH_ZONE` | PolygonZone (142,144)–(72,74)–(2,144) | `RobotHardware` |
| `FAR_LAUNCH_ZONE` | PolygonZone (50,0)–(72,22)–(94,0) | `RobotHardware` |

### Pedro Pathing Constants (`Constants.java`)

| Constant | Default | Tuned by |
|---|---|---|
| `FORWARD_TICKS_TO_INCHES` | 1.0 | Forward Tuner |
| `STRAFE_TICKS_TO_INCHES` | 1.0 | Lateral Tuner |
| `FORWARD_POD_Y` | 0.0 in | Offsets Tuner |
| `STRAFE_POD_X` | 0.0 in | Offsets Tuner |
| `X_VELOCITY` | 1.0 in/s | Forward Velocity Tuner |
| `Y_VELOCITY` | 1.0 in/s | Lateral Velocity Tuner |
| `FORWARD_ZPA` | 50.0 in/s² | Forward ZPA Tuner |
| `LATERAL_ZPA` | 50.0 in/s² | Lateral ZPA Tuner |
| `CENTRIPETAL_SCALING` | 0.005 | Centripetal Tuner |
| `HEADING_P/I/D/F` | 3.0/0.0/0.1/0.0 | Heading Tuner |
| `TRANSLATIONAL_P/I/D/F` | 1.5/0.0/0.05/0.0 | Translational Tuner |
| `DRIVE_P/I/D/F/T` | 0.1/0.0/0.01/0.0/0.6 | Drive Tuner |
| `BRAKING_STRENGTH` | 0.5 | Drive Tuner |
| `ROBOT_MASS_KG` | 5.0 kg | Measured |
| `PATH_MAX_VELOCITY` | 40.0 in/s | Field tuning |
| `PATH_MAX_ACCELERATION` | 60.0 in/s² | Field tuning |
