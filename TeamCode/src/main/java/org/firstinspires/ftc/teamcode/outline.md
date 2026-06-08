# DECODE Robot Code — Architecture Outline

> Comprehensive reference for the FTC DECODE shoot-assist TeleOp. All tuning knobs are
> `public static` in `RobotHardware` so they appear as Panels Configurables and can be
> adjusted live without redeploying.

---

## Table of Contents

1. [Overview](#1-overview)
2. [Hardware Layout](#2-hardware-layout)
3. [Drive Subsystem](#3-drive-subsystem)
4. [Flywheel Subsystem](#4-flywheel-subsystem)
5. [Hood Subsystem](#5-hood-subsystem)
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
  right stick ─────────────────► Drive  (rotation)
  right trigger held ───────────► ALIGNING / ALIGNED
  left trigger (ALIGNED + ready)► SHOOT
  right bumper ─────────────────► INTAKE
  left bumper ──────────────────► INTAKE_REVERSE (king)

Gamepad2
  dpad up/down ────────────────► FlywheelSubsystem.addOffset()
  dpad left/right ─────────────► RobotHardware.hoodAngleOffset
  share held ──────────────────► override isReadyToShoot = true
  left stick btn + right stick btn► Base Zone RTP
  circle ──────────────────────► MasterController → INTAKE (instant)
```

**Libraries used:**

- **SolversLib** — motor/servo wrappers, gamepad helpers, command base classes, telemetry
- **Pedro Pathing** — deadwheel odometry, pose tracking, follower (auto)
- **Marrow** — pose persistence via Settings, PolygonZone for launch/base zone detection
- **Panels** — live tuning via Configurables, field view, graphs
- **Limelight API** (FTC SDK) — AprilTag / MegaTag 2 vision

---

## 2. Hardware Layout

All hardware is declared and initialised in `RobotHardware.init()`.

### Drive Motors

| Motor | Type | Zero Power | Direction |
|-------|------|-----------|-----------|
| FL | `MotorEx(GoBILDA.RPM_435)` | BRAKE | normal |
| FR | `MotorEx(GoBILDA.RPM_435)` | BRAKE | `setInverted(true)` |
| BL | `MotorEx(GoBILDA.RPM_435)` | BRAKE | normal |
| BR | `MotorEx(GoBILDA.RPM_435)` | BRAKE | `setInverted(true)` |

- `GoBILDA.RPM_435` sets `ACHIEVABLE_MAX_TICKS_PER_SECOND = 2769` (384.5 CPR × 435 RPM / 60 s)
- **BRAKE** mode means motors actively resist motion when sticks are centered
- Right side motors are software-inverted so positive power = forward on both sides

### Flywheel Motors

| Motor | Type | Feedforward |
|-------|------|-------------|
| flywheelL | `MotorEx` (VelocityControl) | kS=0.15, kV=12/maxVel |
| flywheelR | `MotorEx` (VelocityControl) | kS=0.15, kV=12/maxVel |

`MotorEx` in `VelocityControl` runs a per-cycle voltage loop with PIDF + torque clamping.

Motor physics constants (GoBilda 435 RPM, 13.7:1 planetary):

| Constant | Value | How derived |
|----------|-------|-------------|
| `FLYWHEEL_TPR` | 384.5 ticks/rev | encoder CPR |
| `FLYWHEEL_OMEGA` | 45.55 rad/s | 435 RPM × 2π/60 |
| `FLYWHEEL_R` | 1.30 Ω | 12 V / 9.2 A stall |
| `FLYWHEEL_V_RES` | 0.326 V | 0.25 A × R |
| `FLYWHEEL_K_EMF` | 0.256 V/(rad/s) | (12 − V_RES) / OMEGA |

### Intake Motor

`Motor` (no encoder). Simple `setPower` with battery voltage compensation — no feedforward, no PID.

### Hood Servos (ServoExGroup)

| Servo | Role | Direction |
|--------|------|-----------|
| hoodL | leader | normal |
| hoodR | follower | `setInverted(true)` |
| hood | ServoExGroup | single handle used by HoodSubsystem |

`hoodR` is hardware-reversed so sending the same position value moves both servos in opposite physical directions, keeping the hood symmetric.

### Gate Servo

```
gate  ServoEx
  GATE_CLOSE_POSITION = 0.0   (closed at init)
  GATE_OPEN_POSITION  = 0.55
```

### IMU

```
IMU  LogoFacingDirection.UP
     UsbFacingDirection.LEFT
```

`getYawRadians()` returns the robot's field heading. Resettable via `imu.resetYaw()` (Touchpad, pose restore, MegaTag re-seed).

### Voltage Sensor

`voltageSensor = hwMap.voltageSensor.get("Control Hub")`

Read every loop via `RobotHardware.batteryVoltage()`. All motor voltages are scaled by `vClamped / batteryVoltage` so performance is consistent from 12 V down to ~10 V.

### Lynx Bulk Caching

```
controlHub = (parent LynxModule)
controlHub.setBulkCachingMode(MANUAL)
```

`hw.clearBulkCache()` is called once per loop to minimise USB round-trips.

---

## 3. Drive Subsystem

**File:** `subsystems/DriveSubsystem.java`
**Parent:** `SolversLib MecanumDrive`

`DriveSubsystem` extends `MecanumDrive` to inherit field-centric kinematics, but **fully overrides `driveFieldCentric()`** to apply input shaping, voltage-based torque control, and stall power reduction before sending power to the motors.

### Control Chain

```
gamepad sticks
    │
    ▼ applyInputCurve(strafe/forward/turn, DRIVE_INPUT_CURVE_EXP)   ← exponential input curve
    │
    ▼ MecanumDrive.driveFieldCentric()
    │  (field-centric rotation, inverse kinematics → 4 wheel fractions)
    ▼
applyVoltageControl(fl, fr, bl, br)   ← overridden in DriveSubsystem
    │
    ├─ vTarget  = frac × 12 V
    ├─ vBackEmf = K_EMF × ω            (opposes applied voltage)
    ├─ iTarget  = |frac| < 0.01 → 0 (coast near zero)
    │              else → MAX_CURRENT    (3 A — torque clamp)
    ├─ vMin = vBackEmf − iTarget × R_MOTOR
    ├─ vMax = vBackEmf + iTarget × R_MOTOR
    ├─ vClamped = clamp(vTarget, vMin, vMax)
    ├─ power = vClamped / batteryVoltage
    ├─ if stallPowerScale < 1.0: power *= stallPowerScale   ← stall reduction
    └─ motor.set(power)
```

### Motor Constants (GoBilda 435 RPM)

| Constant | Value | How derived |
|----------|-------|-------------|
| `TPR` | 384.5 ticks/rev | encoder CPR |
| `I_STALL` | 9.2 A | motor spec |
| `R_MOTOR` | 1.30 Ω | 12 V / 9.2 A |
| `V_RESISTOR` | 0.326 V | 0.25 A no-load × R |
| `K_EMF` | 0.256 V/(rad/s) | (12 − 0.326) / 45.55 |
| `MAX_CURRENT` | **3.0 A** (tunable) | current clamp threshold |
| `STALL_CHECK_INTERVAL` | **10 loops** (tunable) | debounce for stall detection |
| `STALL_POWER_FLOOR` | **0.3** (not tunable) | minimum power during stall |
| `DRIVE_INPUT_CURVE_EXP` | **1.5** (tunable) | joystick sensitivity curve |

### Input Curve

An exponential curve `Math.copySign(Math.pow(|input|, exp), input)` is applied to all three joystick axes before clipping. `exp = 1.0` is linear; values above 1.0 give more sensitivity near zero.

### Stalling Detection + Power Reduction

`applyVoltageControl()` monitors motor current every loop. When any motor exceeds `MAX_CURRENT`:

- `stallPowerScale` drops 15% per tick, clamped to `STALL_POWER_FLOOR = 0.3`
- All wheel powers are multiplied by `stallPowerScale`

When the stall clears, `stallPowerScale` recovers at +10% per interval check (10 ticks), giving a smooth ramp rather than a hard cut.

`isStalling()` returns `true` if `stallPowerScale < 1.0`.

### Zero Power Behavior

All four drive motors are **BRAKE** — when power = 0 they resist motion rather than coasting.

---

## 4. Flywheel Subsystem

**File:** `subsystems/FlywheelSubsystem.java`

Two motors in `VelocityControl` mode, updated every loop with a **manual PIDF + back-EMF compensation + torque limiting** loop.

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

### Tunable Constants

| Constant | Default | Meaning |
|----------|---------|---------|
| `FLYWHEEL_TARGET_VELOCITY` | 2800 ticks/s | speed both motors try to reach |
| `FLYWHEEL_READY_TOLERANCE` | 150 ticks/s | `isReadyToShoot` velocity threshold |
| `FLYWHEEL_L_KP/KI/KD/KF` | 0.0001 / 0.001 / 0 / 0 | left motor PID |
| `FLYWHEEL_R_KP/KI/KD/KF` | 0.0001 / 0.001 / 0 / 0 | right motor PID |
| `FLYWHEEL_MAX_CURRENT` | 3.0 A | torque clamp — protects motors under load |
| `flywheelVelocityOffset` | 0 (live) | gamepad2 dpad up/down delta ±50 |

Left and right have separate PID gains to handle slight mechanical differences. `addOffset(delta)` adjusts `targetVelocity` live; `reset()` zeroes integral and derivative history.

---

## 5. Hood Subsystem

**File:** `subsystems/HoodSubsystem.java`

### Single-Value Control via ServoExGroup

Both servos physically mirror each other (one reversed) so a single `setPosition(p)` moves the entire hood. `ServoExGroup` broadcasts the leader's position to all followers.

### Hardstop Enforcement

`clamp(position, HOOD_MIN_POSITION, HOOD_MAX_POSITION)` is applied before every write.

| Constant | Value |
|----------|-------|
| `HOOD_MIN_POSITION` | 0.05 |
| `HOOD_MAX_POSITION` | 0.80 |
| `HOOD_ANGLE_OFFSET_JUMP` | 0.02 (dpad left/right increment) |

### Hood LUT (Aiming)

`RobotHardware.HOOD_LUT` is a linear-interpolation lookup table from measured distance (inches) to servo position.

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

`Motor` (no encoder). Simple `setPower` with battery voltage compensation. The commanded voltage is scaled by `12 / batteryVoltage` so the intake runs at consistent speed regardless of battery sag. No PID, no feedforward model.

### Voltage Compensation (per cycle)

```
power = INTAKE_POWER / batteryVoltage
motor.set(command × power)
```

`INTAKE_POWER` is in volts (default 12 V). Tune this to change intake speed.

### Commands

| Method | Effect |
|--------|--------|
| `runForward()` | Drive intake forward at `INTAKE_POWER` V |
| `runReverse()` | Drive intake reverse at `INTAKE_POWER` V |
| `stop()` | Cut motor output immediately |
| `isRunning()` | `true` if `motor.get() > 0.01` |

### Tunable Constants

| Constant | Default | Meaning |
|----------|---------|---------|
| `INTAKE_MAX_CURRENT` | 4.0 A | current clamp — protects motor under jam |
| `INTAKE_MAX_VELOCITY` | ~2005 ticks/s | derived: TPR × RPM / 60 |

---

## 7. Gate Subsystem

**File:** `subsystems/GateSubsystem.java`

Single `ServoEx`. `isOpen()` returns true if `gate.getPosition() > (OPEN + CLOSE) / 2`.

| Position | Value | Use |
|----------|-------|-----|
| `GATE_CLOSE_POSITION` | 0.0 | Default at init; closed during intake and alignment |
| `GATE_OPEN_POSITION` | 0.55 | Opened during `ShootCommand` |

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
2. **Right bumper (gamepad1)** → `INTAKE`
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
**File:** `commands/BaseZoneRTPCommand.java`

### Launch Zone

Two Marrow `PolygonZone` instances define the pull region. The robot footprint is represented by `RobotHardware.ROBOT_ZONE` (a `ROBOT_SIZE_INCHES` × `ROBOT_SIZE_INCHES` square), which is synced to the live pose every loop.

| Zone | Vertices (Pedro coords) | Shape |
|------|------------------------|-------|
| `CLOSE_LAUNCH_ZONE` | (144,144)–(72,72)–(0,144) | large right triangle in scoring corner |
| `FAR_LAUNCH_ZONE` | (58,0)–(72,24)–(96,0) | small triangle near field center |

**Active when:** gamepad1 right trigger held AND state = `ALIGNING` or `ALIGNED` AND robot footprint is not even partially inside either zone (checked via `ROBOT_ZONE.isInside()`).

### Base Zone

Two Marrow `PolygonZone` instances define the pull target:

| Zone | Center | Size | Alliance |
|------|--------|------|---------|
| `BLUE_BASE_ZONE` | (105.5, 33.5) | 20×20 in | BLUE |
| `RED_BASE_ZONE` | (38.5, 33.5) | 20×20 in | RED |

**Active when:** gamepad2 left stick button + right stick button held.

### Algorithm (Launch Zone)

1. **Zone selection:** compute `CLOSE_ZONE.distanceTo(robot)` and `FAR_ZONE.distanceTo(robot)`; pick the nearer zone
2. **Closest-point-on-polygon:** iterate every edge; find the closest point on that edge to the robot; return the minimum
3. **Pull vector:** unit vector from robot → closest boundary point
4. **World-frame driver input:** rotate joystick by `robotH`
5. **Blend:** `blended = pull × (1 − WEIGHT) + driverWorld × WEIGHT`; `VECTOR_WEIGHT_DRIVER = 0.6`
6. **Normalize** to unit magnitude

### Tuning

| Constant | Default | Effect |
|----------|---------|--------|
| `VECTOR_WEIGHT_DRIVER` | 0.6 | Driver authority — higher = driver has more override |

---

## 11. Distance Measurement

### How Distance Is Computed

```
dx = GOAL_COORDS.x − robotX
dy = GOAL_COORDS.y − robotY
distance = √(dx² + dy²)
```

`robotX / Y / H` are the Kalman-filtered pose (Pedro `Follower.getPose()` after fusing odometry + MegaTag 2 vision). `GOAL_COORDS` is switched per-alliance from `RED_GOAL_COORDS` / `BLUE_GOAL_COORDS` in `RobotHardware`.

Rejected if `distance < LIMELIGHT_DIST_MIN (5 in)` or `> LIMELIGHT_DIST_MAX (120 in)`.

### Uses

- **Hood aiming** — `hoodPosition(distance)` → interpolates from `HOOD_DISTANCE_SAMPLES` → `HOOD_POSITION_SAMPLES` lookup table
- **`isReadyToShoot`** — requires a valid pose distance to allow shooting
- **Zone check** — `ROBOT_ZONE.isInside()` via Marrow; shooting is enabled when the robot footprint is partially or fully inside either launch zone, and blocked when fully outside both
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
|----------|-------|---------|
| RED | 0 | RED |
| BLUE | 1 | BLUE |

Switched during `init_loop()` alliance selection and locked at `start()`.

---

## 12. Localization — Pedro Pathing + Kalman Filter

### Pedro Pathing

```
follower = Constants.createFollower(hardwareMap)
follower.update()   // every loop
follower.getPose() → Pose(x, y, heading)
```

Deadwheel odometry (forward pod on FL motor, lateral pod on BR motor) gives continuous pose between vision updates. `followerd.startTeleopDrive()` is **not called** — Pedro provides pose estimation only; `DriveSubsystem` handles all TeleOp motor writes.

### IMU Integration

`limelight.updateRobotOrientation(hw.getYawRadians())` is called **before** `follower.update()` every loop. This feeds the raw IMU yaw into Pedro's TwoWheelLocalizer so it can integrate gyro heading with the deadwheel encoder deltas.

### Kalman Drift Filter (`DriftFilter` class in RobotHardware)

Three independent 1-D Kalman filters, one per axis (X, Y, Heading).

**Predict step** (every loop, with timestep `dt`):
```
P += qBias² × dt      // covariance grows (random walk / gyro drift)
```

**Update step** (when valid MegaTag 2 reading is available):
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

The **first measurement after a pose reset** bypasses the staleness check (via `localizerJustReset` flag) so the filters anchor immediately without accumulating uncertainty.

| Tunable | Default | Meaning |
|---------|---------|---------|
| `KALMAN_Q_BIAS_X/Y/H` | 0.02 / 0.02 / 0.005 | process noise (variance per √s) |
| `KALMAN_R_VISION_X/Y/H` | 4.0 / 4.0 / 0.01 | measurement noise variance |

### Pose Getters Used Across the Code

```
getCorrectedX(odomX) → Kalman-corrected X
getCorrectedY(odomY) → Kalman-corrected Y
getCorrectedH(odomH) → Kalman-corrected heading
```

Used by: Launch Zone RTP, odometry heading fallback for auto-align, zone containment checks, telemetry.

---

## 13. Driver Controls (Gamepads)

All raw reads go through `GamepadEx` from SolversLib for edge detection.

### Gamepad 1 (Driver)

| Input | Action |
|-------|--------|
| Left stick X/Y | Forward/strafe drive |
| Right stick X | Rotation |
| Right trigger (`> 0.5`) | Hold → `ALIGNING` |
| Left trigger (edge) | Fire shot (requires `ALIGNED` + `isReadyToShoot`) |
| Right bumper | `INTAKE` |
| Left bumper | `INTAKE_REVERSE` (king — overrides everything) |
| Triangle | Select RED alliance (in `init_loop`) |
| Circle | Select BLUE alliance (in `init_loop`) |
| Options | Re-zero field-centric drive yaw to current heading (software offset, no IMU reset) |
| Touchpad | Reset pose to `(72, 144, allianceYaw)` + reinit Kalman + 200 ms rumble |
| Share | Re-seed pose from MegaTag vision (edge-triggered) |

### Gamepad 2 (Operator)

| Input | Action |
|-------|--------|
| Dpad up | Increase flywheel velocity offset (+50 ticks/s) |
| Dpad down | Decrease flywheel velocity offset (−50 ticks/s) |
| Dpad left | Increase hood angle offset (+0.02) |
| Dpad right | Decrease hood angle offset (−0.02) |
| Share held | Override `isReadyToShoot = true` (edge: 2 haptic blips on rising edge) |
| Left Stick Button + Right Stick Button | Enable Base Zone RTP |
| Circle | Instant `INTAKE` (disregards all other state) |

### isReadyToShoot

All five conditions must be true:

1. State is `ALIGNED`
2. Limelight has a valid target
3. Both flywheel velocities within `FLYWHEEL_READY_TOLERANCE` of target
4. Pose distance is valid
5. Robot footprint is **partially or fully inside** either launch zone — `ROBOT_ZONE.isInside(CLOSE_LAUNCH_ZONE)` / `ROBOT_ZONE.isInside(FAR_LAUNCH_ZONE)` checked via Marrow. Shooting is blocked when fully outside both zones.

When `isReadyToShoot` changes from false → true (rising edge), both gamepads receive the **50ms-on / 50ms-pause / 50ms-on** rumble pattern. `gamepad2.share` override fires its own separate 2-blip rumble on its rising edge.

---

## 14. Telemetry — Panels

`RobotHardware.panels` holds the `Panels` instance. `RobotHardware.telemetryData` wraps `panels.getTelemetry()`.

Displayed every loop:

- Alliance, State
- Flywheel velocity (L, R, target)
- Gate position, Hood position
- `isReadyToShoot`, Share override
- Limelight distance, `tx`, `ty`
- Pose (X, Y, heading in degrees)
- Flywheel velocity offset, Hood angle offset
- Base Zone RTP active, Launch Zone RTP active

All constants are `public static` in `RobotHardware` annotated with `@Configurable`, so they appear as live-tuning sliders in the Panels dashboard.

---

## 15. Autonomous — MainAuto

**File:** `MainAuto.java`

Autonomous uses **Pedro Pathing** for all drive movement. `DriveSubsystem` is **not used** in auto — Pedro handles motor writes internally.

### Pedro Pathing Setup

```
Constants.createFollower(hardwareMap)
  └─ TwoWheelLocalizer    ← forward pod (FL motor) + lateral pod (BR motor) + IMU
  └─ MecanumDrivetrain    ← FL / BL / FR / BR motors
  └─ FollowerConstants    ← mass, PIDF, velocity, ZPA, centripetal
  └─ PathConstraints      ← maxVel, maxAccel, maxJerk, maxAngVel
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

**IMU orientation** — set `IMU_LOGO_FACING` and `IMU_USB_FACING` in `Constants.java` (default: UP + LEFT).

MegaTag 2 runs continuously during auto, updating the Kalman filter so the `Follower`'s pose stays accurate.

---

## Constants Quick Reference

### RobotHardware

| Constant | Value | Notes |
|----------|-------|-------|
| `FLYWHEEL_TARGET_VELOCITY` | 2800 ticks/s | |
| `FLYWHEEL_READY_TOLERANCE` | 150 ticks/s | |
| `FLYWHEEL_MAX_CURRENT` | 3.0 A | |
| `FLYWHEEL_L_KP/KI/KD/KF` | 0.0001 / 0.001 / 0 / 0 | |
| `FLYWHEEL_R_KP/KI/KD/KF` | 0.0001 / 0.001 / 0 / 0 | |
| `FLYWHEEL_TPR / RPM` | 384.5 / 435 | |
| `FLYWHEEL_K_EMF` | 0.256 V/(rad/s) | |
| `FLYWHEEL_R` | 1.30 Ω | |
| `INTAKE_POWER` | 12.0 V | intake motor voltage (battery-compensated) |
| `SHOOT_DELAY` | 2.5 s | |
| `GATE_OPEN_POSITION` | 0.55 | |
| `GATE_CLOSE_POSITION` | 0.0 | |
| `HOOD_MIN/MAX_POSITION` | 0.05 / 0.80 | |
| `HOOD_COMPENSATION_COEFFICIENT` | 0.0005 | kH factor |
| `DRIVE_MAX_CURRENT` | 3.0 A | |
| `DRIVE_STALL_CURRENT_THRESHOLD` | 2.5 A | |
| `DRIVE_INPUT_CURVE_EXP` | 1.5 | |
| `ALIGNMENT_DELAY` | 0.15 s | |
| `VECTOR_WEIGHT_DRIVER` | 0.6 | |
| `LIMELIGHT_MOUNT_ANGLE` | 25° | |
| `LIMELIGHT_DIST_MIN/MAX` | 5 / 120 in | |
| `RED_GOAL_COORDS` | (144, 144) | Pedro frame |
| `BLUE_GOAL_COORDS` | (0, 144) | Pedro frame |
| `CLOSE_LAUNCH_ZONE` | PolygonZone (142,144)–(72,74)–(2,144) | Marrow |
| `FAR_LAUNCH_ZONE` | PolygonZone (50,0)–(72,22)–(94,0) | Marrow |
| `BLUE_BASE_ZONE` | PolygonZone 20×20 at (105.5, 33.5) | Marrow |
| `RED_BASE_ZONE` | PolygonZone 20×20 at (38.5, 33.5) | Marrow |
| `KALMAN_Q_BIAS_X/Y/H` | 0.02 / 0.02 / 0.005 | |
| `KALMAN_R_VISION_X/Y/H` | 4.0 / 4.0 / 0.01 | |

### Pedro Pathing Constants (`Constants.java`)

| Constant | Default | Tuned by |
|----------|---------|---------|
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
