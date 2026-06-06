# DECODE Robot — Tuning Guide

> All tuning constants are `public static` in `RobotHardware` or in the relevant subsystem class.
> Most are live-adjustable via **Panels Configurables** without redeploying.
> A `-->` indicates "change this value, redeploy, and test."

---

## Table of Contents

1. [Before You Start](#1-before-you-start)
2. [Flywheel Motors](#2-flywheel-motors) — velocity PIDF, target velocity
3. [Intake Motor](#3-intake-motor) — current limit
4. [Drive Motors](#4-drive-motors) — current limit, input curve, IMU
5. [Hood Servos](#5-hood-servos) — hardstops, LUT, kH compensation, dpad offsets
6. [Gate Servo](#6-gate-servo) — open/close positions
7. [Limelight](#7-limelight) — mount angle, goal height, distance offset
8. [Auto-Alignment](#8-auto-alignment) — on-target threshold, alignment delay, rotation P
9. [Kalman Drift Filter](#9-kalman-drift-filter) — Q_BIAS and R_VISION per axis
10. [Launch Zone RTP](#10-launch-zone-rtp) — vector weight, launch zone offset
11. [Shoot Sequence](#11-shoot-sequence) — shoot delay
12. [Pedro Pathing](#12-pedro-pathing) — path constraints

---

## 1. Before You Start

### Tools

- **REV Hardware Client** — view live motor current, velocity, and voltage
- **Panels dashboard** — live-adjust tunables (most constants are `@Configurable`)
- **DS telemetry** (driver station phone or laptop) — basic telemetry readout
- A tape measure and a known alliance goal position
- A second driver station phone for gamepad 2

### Tuning Philosophy

Tune in this order — later stages depend on earlier ones:

```
Flywheel velocity → Hood LUT → Gate → Intake → Drive → Limelight → Alignment → Kalman → RTP
```

Start each parameter at its default. Change one thing at a time. Write down the result before moving on.

### Reading Motor Current

In the REV Hardware Client:
1. Connect to the robot (Live with robot powered)
2. Expand the Motor list
3. Click a motor → **Current** tab

Or add this to `sendTelemetry()` in `MainTeleOp` for live readout:

```java
telemetryData.addData("FL current (A)", hw.fl.getCurrent(CurrentUnit.AMPS));
```

---

## 2. Flywheel Motors

**Location:** `RobotHardware` — `FLYWHEEL_*` constants + `FlywheelSubsystem`

### 2.1. `FLYWHEEL_TARGET_VELOCITY` — Target Speed

**What it does:** Sets the speed (in ticks/sec) that both flywheel motors try to reach.
**Default:** 2800
**How to measure:** Run the robot near the goal, hold ALIGNING, and observe the `FlywheelL vel` and `FlywheelR vel` telemetry values. Adjust until the note launches cleanly.

**Tuning steps:**
1. Start at 2600 ticks/s
2. Launch a note — does it clear the goal height?
3. If yes, increase by 100 ticks/s and repeat
4. The correct value is the **lowest speed that consistently scores** — overshooting wastes battery and strains the motor

> **Overshoot test:** If the note consistently lands beyond the goal or bounces off the back wall, speed is too high.

### 2.2. `FLYWHEEL_L_KP` / `FLYWHEEL_R_KP` — Proportional Gain

**What it does:** Reacts to the instantaneous velocity error. Higher = faster correction, more jitter.
**Default:** 0.0001

**Tuning steps:**
1. Start with `KI = KD = KF = 0`
2. Set `KP = 0.0003` — watch telemetry for oscillation (velocity bouncing above and below target)
3. If it oscillates, halve `KP` until steady
4. If it settles too slowly (takes > 1 s to reach target after a load bump), double `KP`

> **Oscillation:** Velocity swings more than ±100 ticks/s above and below target — reduce `KP` by half.

### 2.3. `FLYWHEEL_L_KI` / `FLYWHEEL_R_KI` — Integral Gain

**What it does:** Eliminates steady-state error (flywheel always running slightly below target).
**Default:** 0.001

**Tuning steps:**
1. With `KP` already set, observe `FlywheelL vel` in steady state
2. If steady-state error is consistently > 50 ticks/s below target, increase `KI` by 0.0005
3. Cap `KI` so the integral term does not cause overshoot on start-up

> **Integral windup:** If the flywheel overshoots on start-up and takes a long time to recover, reduce `KI` or lower the anti-windup clamp (currently ±12 V — edit `applyVoltageLoop()` in `FlywheelSubsystem`).

### 2.4. `FLYWHEEL_L_KF` / `FLYWHEEL_R_KF` — Feedforward Velocity Constant

**What it does:** Adds a fixed voltage term proportional to target velocity. Higher = more voltage fed forward at target speed.
**Default:** 0.0 (back-EMF is handled by the voltage loop directly)

**Tuning steps:**
1. Usually leave at 0 with the voltage loop
2. If you want the flywheel to start faster, try `KFF = 0.00005` and re-tune `KP`/`KI`

### 2.5. `FLYWHEEL_MAX_CURRENT` — Current Clamp

**What it does:** Limits how hard the motor works. Higher = more torque available, more heat.
**Default:** 3.0 A
**Range:** 1.5 A – 6.0 A

**Tuning steps:**
1. Set to 2.0 A — does the flywheel still reach target velocity? Can it score?
2. If yes, the flywheel load is low enough; keep at 2.0 A for longevity
3. If the flywheel cannot reach target speed (drops significantly under note load), increase in 0.5 A steps

> **Heat test:** After a full match, touch the motor housing. If it's too hot to hold for > 3 s, reduce `MAX_CURRENT` by 0.5 A.

### 2.6. Live Offset — `FLYWHEEL_VELOCITY_OFFSET_JUMP`

**What it does:** Gamepad2 dpad up/down increments the live velocity offset by this amount.
**Default:** 50 ticks/s per press

**Tuning steps:**
- No tuning needed — adjust to taste during a match
- If you find yourself needing large corrections (> ±200 ticks/s), re-tune `FLYWHEEL_TARGET_VELOCITY`

---

## 3. Intake Motor

**Location:** `RobotHardware` — `INTAKE_MAX_CURRENT`

### 3.1. `INTAKE_MAX_CURRENT` — Current Limit

**What it does:** Clamps voltage so the intake motor never draws more than this current.
**Default:** 4.0 A
**Range:** 2.0 A – 7.0 A

**Tuning steps:**
1. Run the intake at full speed (gamepad1 right bumper held)
2. Watch current in **REV Hardware Client** (expand the intake motor → Current tab)
3. Normal running current should be 1–2 A
4. Jam the intake with a note — current should **not** exceed `INTAKE_MAX_CURRENT`
5. If current spikes above 4 A under jam, reduce `INTAKE_MAX_CURRENT` by 0.5 A
6. If the intake feels weak even on an unobstructed run, increase `INTAKE_MAX_CURRENT`

> **Reverse test:** Hold gamepad1 left bumper to run `INTAKE_REVERSE`. Confirm it runs smoothly in both directions. If one direction is weaker, one motor may need its direction inverted in the hardware map.

### 3.2. `INTAKE_MAX_VELOCITY` / `INTAKE_RPM`

**What it does:** Sets the nominal max velocity used in the pure feedforward calculation (`ω_command = command × maxVel / TPR × 2π`). The motor runs open-loop — this value is the assumed no-load speed at 12 V; actual speed will vary with load and battery sag.
**Default:** 312 RPM → ~2005 ticks/s

**When to change:** Update `INTAKE_RPM` if you swap the motor for a different gear ratio. `INTAKE_MAX_VELOCITY` recomputes automatically.

**Note on encoder:** Since the intake has no encoder, there is no velocity feedback loop. The only way to adjust intake speed in real time is via the current limit (`INTAKE_MAX_CURRENT`). If the intake consistently feels underpowered, reduce the current limit slightly; if it stalls too easily on notes, increase it.

---

## 4. Drive Motors

**Location:** `DriveSubsystem` — `MAX_CURRENT`, `STALL_CHECK_INTERVAL`; `RobotHardware` — `DRIVE_INPUT_CURVE_EXP`

### 4.1. `MAX_CURRENT` (DriveSubsystem) — Voltage Clamp Current

**What it does:** The voltage loop clamps the applied voltage so the motor never needs to source more than this current. Higher = more torque available, more heat.
**Default:** 3.0 A
**Range:** 1.5 A – 6.0 A

**Tuning steps:**
1. Drive the robot at full speed on carpet — watch motor current in REV Hardware Client
2. Typical driving current: 0.5–2.0 A
3. If current **never** reaches 3 A even when pushing against a wall, the default is safe and you could try 4.0 A for more punch
4. If current frequently hits 3 A under normal driving (hills, heavy driver, pushing), reduce to 2.5 A

> **Pushing test:** Drive into a wall at full forward. If the robot barely moves and the motors hum loudly (> 3 A), `MAX_CURRENT` is working as intended.

### 4.2. `STALL_CHECK_INTERVAL` (DriveSubsystem) — Stall Debounce

**What it does:** How many loop iterations between clearing the stall flag when current drops below threshold.
**Default:** 10

**When to change:**
- Lower (5): more responsive stall detection — good for triggering LEDs or warnings
- Higher (20–30): more debounce — good for filtering false positives from acceleration spikes

### 4.3. `DRIVE_STALL_CURRENT_THRESHOLD` (RobotHardware) — Stall Trigger

**What it does:** The raw threshold used by `isDriveStallingAny()`. Set slightly below `MAX_CURRENT` so the stall flag fires before the voltage clamp activates.
**Default:** 2.5 A
**Range:** 1.0 A – 5.0 A

**Tuning steps:**
1. Watch current under normal driving (should be well below threshold)
2. Set to 2.5 A
3. Push against a wall — the stall flag should activate just before the voltage clamp limits power
4. If the stall flag activates during normal driving, raise the threshold
5. If the stall flag never activates even when the robot is stuck, lower the threshold

### 4.4. `DRIVE_INPUT_CURVE_EXP` (RobotHardware) — Joystick Response

**What it does:** Applies an exponential curve to joystick input. `1.0` = linear; `> 1.0` = more sensitivity near zero.
**Default:** 1.5

**Tuning steps:**
1. Drive with `EXP = 1.0` (linear) — note how the robot feels
2. Increase to `1.3–1.7` — robot should feel more responsive at low speed (e.g., scoring station alignment)
3. If `EXP` is too high (> 2.0), the robot becomes twitchy and hard to drive slowly
4. Tune to driver preference — aggressive drivers prefer higher; cautious drivers prefer lower

> **Test course:** Set up two traffic cones 18 inches apart. Time how many runs it takes to consistently thread through without touching.

### 4.5. IMU Calibration

**What it does:** The IMU provides yaw (heading) for field-centric drive and alignment.
**Default:** Orientation = logo UP, USB LEFT

**Verification:**
1. Place the robot facing a known wall
2. Press Options on gamepad1 — IMU yaw resets to 0
3. Turn the robot exactly 90° (use a reference)
4. Telemetry should read ~90°

**If the IMU reads backwards:**
Change the `UsbFacingDirection` in `RobotHardware.init()`:

```java
// Try LEFT if you have UP, or DOWN, or RIGHT:
RevHubOrientationOnRobot.UsbFacingDirection.LEFT
```

**If the IMU drifts over time:**
- Calibrate before each match: spin the robot 540° in each direction before INIT ends
- If drift is severe (> 5°/min), the IMU may need replacement

---

## 5. Hood Servos

**Location:** `RobotHardware` — `HOOD_*` constants; `HoodSubsystem`

The hood aims the shooter by mapping distance to a servo position. Tune this on the field with a known goal position.

### 5.1. `HOOD_MIN_POSITION` / `HOOD_MAX_POSITION` — Hardstops

**What it does:** Software limits that prevent the hood from driving into physical stops.
**Default:** 0.05 / 0.80

**Tuning steps:**
1. Set `HOOD_MIN = 0.0`, `HOOD_MAX = 1.0`
2. Drive to a known close range (e.g., 30 inches from goal)
3. Set `setRaw(0.0)` — watch the hood. It should be at its lowest physical angle
4. Record the raw value where it first contacts the hardstop — set `HOOD_MIN` to that + 0.01
5. Repeat at `setRaw(1.0)` for the top of travel → `HOOD_MAX − 0.01`

> **Wrong direction test:** If `setRaw(1.0)` moves the hood downward, swap the servo connections on the servo board, or add `setInverted(true)` on the appropriate `ServoEx` in `RobotHardware.init()`.

### 5.2. `HOOD_DISTANCE_SAMPLES` / `HOOD_POSITION_SAMPLES` — Aim LUT

**What it does:** Maps measured distance to hood servo position. The code linear-interpolates between samples.
**Default:** 6 sample pairs (30→0.75, 40→0.65, 50→0.55, 60→0.45, 70→0.35, 80→0.25)

**Tuning steps:**
1. Place the robot at **30 inches** from the goal center
2. In teleop, watch the hood position telemetry
3. Manually adjust `HOOD_POSITION_SAMPLES[0]` until the note scores consistently
4. Repeat for **50 inches**, **70 inches** (and any distance in between you want to score from)
5. Fill in intermediate distances by interpolating

**Adding more samples:**
Add entries to both arrays — the code handles any length:

```java
public static double[] HOOD_DISTANCE_SAMPLES = {
    24.0,  30.0,  40.0,  50.0,  60.0,  70.0,  80.0,  96.0
};
public static double[] HOOD_POSITION_SAMPLES = {
    0.80,  0.75,  0.65,  0.55,  0.45,  0.35,  0.25,  0.18
};
```

**If you only have a few sample points:**
- Fewer samples → more interpolation error at untested distances
- Target at least 3 samples across the range you intend to shoot from

### 5.3. `HOOD_COMPENSATION_COEFFICIENT` — kH Velocity Drop Compensation

**What it does:** When a note loads, the flywheel decelerates slightly. This coefficient adds extra hood angle proportional to the velocity drop:

```
extraAngle = (targetVelocity − actualVelocity) × kH
```

**Default:** 0.0005

**Tuning steps:**
1. Load a note while watching `Flywheel target` and `FlywheelL vel` on telemetry
2. Note the velocity drop (e.g., target=2800, actual=2600 → drop=200)
3. Estimate the hood angle change needed to compensate:
   - Shoot without note loaded (baseline) → note lands at distance X
   - Load note → note lands at distance Y
   - If Y < X (lands short), increase `kH`
   - If Y > X (overshoots), reduce `kH`

4. Typical range: **0.0002 – 0.0010**
5. Check at multiple distances — a coefficient that works at 40 inches may not work at 70 inches

> **Signs kH is wrong:**
> - Too low: note lands consistently short after loading → increase `kH`
> - Too high: note lands consistently long/overhoots after loading → reduce `kH`

### 5.4. `HOOD_ANGLE_OFFSET_JUMP` — Dpad Increment

**What it does:** Gamepad2 dpad left/right adjusts the live `hoodAngleOffset` by this amount per press.
**Default:** 0.02
**Range:** 0.005 – 0.05

**Tuning steps:**
- If you find yourself needing large corrections (> ±0.10 total), re-tune the LUT instead
- Smaller increments (0.005) allow finer field tuning but require more button presses

### 5.5. `LIMELIGHT_DISTANCE_OFFSET` — Distance Calibration

**What it does:** Adds a fixed offset (inches) to every Limelight distance reading.
**Default:** 0.0

**Tuning steps:**
1. Park the robot at exactly 60 inches from the goal center (measure from the field tile)
2. Observe `Limelight dist (in)` in telemetry
3. If it reads 58 inches: set `LIMELIGHT_DISTANCE_OFFSET = +2.0`
4. If it reads 62 inches: set `LIMELIGHT_DISTANCE_OFFSET = −2.0`
5. Repeat until distance reading matches measured distance

---

## 6. Gate Servo

**Location:** `RobotHardware` — `GATE_OPEN_POSITION`, `GATE_CLOSE_POSITION`

### 6.1. `GATE_CLOSE_POSITION` — Closed Position

**What it does:** Servo position when the gate is closed.
**Default:** 0.0

**Tuning steps:**
1. Run `gate.setPosition(0.0)` in INIT
2. Confirm the note cannot pass through the gate under gravity
3. If the note leaks through: increase to 0.05, 0.08, etc.
4. If the gate servo strains audibly when closed, the position is past the physical stop — reduce it

### 6.2. `GATE_OPEN_POSITION` — Open Position

**What it does:** Servo position during the shoot sequence.
**Default:** 0.55

**Tuning steps:**
1. During `SHOOT`, the gate should open just enough to let the note through without wobbling
2. If the note jams: increase to 0.60, 0.65
3. If the gate overshoots and takes too long to close (note bleeds through): reduce to 0.50, 0.48
4. The correct value is the **minimum position that reliably clears the note**

> **Close speed test:** Fire 5 notes in a row without pausing. All 5 should score. If any leak back or jam, the gate timing (`SHOOT_DELAY`) or position needs adjustment.

---

## 7. Limelight

**Location:** `RobotHardware` — `LIMELIGHT_*` constants

### 7.1. `LIMELIGHT_MOUNT_ANGLE` — Camera Tilt

**What it does:** The angle of the Limelight camera above horizontal (degrees).
**Default:** 25°

**Tuning steps:**
1. Measure the actual mounting angle from the Limelight housing to the floor, then to the goal center
2. Or: park the robot at exactly 60 inches from the goal, observe `Limelight dist (in)`
3. If distance is wrong, adjust `LIMELIGHT_MOUNT_ANGLE` until the distance reading is accurate
4. Small adjustments (±2°) are typical for mechanical mounting variations

### 7.2. `GOAL_HEIGHT` — Goal Height

**What it does:** The height of the goal center above the floor (inches), used in the distance trigonometry.
**Default:** 18.0 inches

**Tuning steps:**
1. Check the game manual for the official goal height
2. Or: measure from floor to center of the goal ring
3. Update `GOAL_HEIGHT` if the manual measurement differs from 18 inches

### 7.3. Pipeline Indices

**What it does:** Tells the code which Limelight pipeline to use for each alliance.
**Default:** RED = 0, BLUE = 1

**Tuning steps:**
1. In Limelight Web UI → Pipelines, assign your red AprilTag pipeline to index 0 and blue to index 1
2. Confirm in `init_loop()` that pressing Triangle/Circle switches pipelines correctly (telemetry shows alliance change)

### 7.4. `LIMELIGHT_DIST_MIN` / `LIMELIGHT_DIST_MAX` — Range Guards

**What it does:** Rejects Limelight distance readings outside this range to prevent garbage data.
**Default:** 5.0 / 120.0 inches

**Tuning steps:**
1. Drive the robot to the closest shooting position and confirm distance reads
2. Set `MIN` just inside the closest valid shooting distance
3. Set `MAX` to the farthest distance you intend to shoot from

---

## 8. Auto-Alignment

**Location:** `RobotHardware` — `ALIGNMENT_DELAY`; `MainTeleOp` — rotation P gain

### 8.1. On-Target Threshold — `MasterController.java` line 214

```java
double onTargetThreshold = 2.0; // degrees — tune via RobotHardware
```

**What it does:** How close the limelight crosshair (`tx`) must be to the goal before the alignment timer starts.
**Default:** 2.0°

**Tuning steps:**
1. Set to 1.0° — alignment is stricter; robot must be very precise
2. If drivers struggle to hold alignment: increase to 3.0°
3. If the robot transitions to ALIGNED but the shot misses: reduce to 1.5° or 1.0°

### 8.2. `ALIGNMENT_DELAY` — Dwell Time

**What it does:** Seconds the limelight crosshair must stay within the on-target threshold before transitioning from ALIGNING → ALIGNED.
**Default:** 0.15 s

**Tuning steps:**
1. Set to 0.0 s — robot transitions immediately when `tx` is within threshold
2. If alignment flickers on and off rapidly: increase to 0.10–0.20 s
3. If the robot takes too long to lock on (drivers frustrated): decrease to 0.05 s
4. Higher `ALIGNMENT_DELAY` + stricter `onTargetThreshold` = more robust alignment; lower both = faster but more jittery

### 8.3. Rotation P Gain — `MainTeleOp.java` line 296

```java
rotationCorrection = result.getTx() * 0.05; // P on crosshair
```

**What it does:** P-controller gain on the limelight crosshair offset. Higher = faster correction, more overshoot.
**Default:** 0.05

**Tuning steps:**
1. Set to `0.03` — slower, smoother correction
2. If the robot oscillates left-right when aligning: reduce to `0.02`
3. If the robot is slow to correct and drifts far off-center before correcting: increase to `0.07–0.10`
4. The P gain for the **odometry fallback** (when no target is visible) uses the same constant

---

## 9. Kalman Drift Filter

**Location:** `RobotHardware` — `KALMAN_Q_BIAS_*`, `KALMAN_R_VISION_*`

The Kalman filter runs in two modes: **predict** (every loop, covariance grows) and **update** (when MegaTag 2 gives a fresh reading).

Two parameters per axis (X, Y, Heading):
- **Q_BIAS** — process noise. How fast the drift grows over time. Higher = filter trusts vision more quickly.
- **R_VISION** — measurement noise. How noisy the vision reading is. Higher = filter trusts vision less.

### 9.1. Quick Test

1. Park the robot. Watch `Pose X` in telemetry over 30 seconds without moving.
2. If it drifts significantly (> 5 inches): Q_BIAS is too high **or** R_VISION is too low
3. If it snaps to a different value every vision update: R_VISION is too low

### 9.2. `KALMAN_R_VISION_X` / `_Y` — Vision Trust

**What it does:** Variance of the MegaTag 2 pose estimate. Lower = more trust in vision.
**Default:** 4.0 (inches²)

**Tuning steps:**
1. Set to `2.0` — filter trusts vision more → pose is smoother but slower to correct on large jumps
2. If the robot's displayed pose "jumps" whenever a new AprilTag comes into view: increase to `6.0–10.0`
3. If the pose slowly drifts away from reality and vision corrections are too slow: reduce to `1.0–2.0`

### 9.3. `KALMAN_Q_BIAS_X` / `_Y` — Drift Growth Rate

**What it does:** Random-walk rate for position drift. Higher = faster drift accumulation → vision corrections are stronger.
**Default:** 0.02

**Tuning steps:**
1. Set to `0.01` — slower drift growth, more stable pose but slower to correct
2. If the robot starts a match and by endgame the pose has drifted > 10 inches: increase to `0.03–0.05`
3. If the pose "snaps" back too aggressively when a vision reading arrives: reduce Q_BIAS

### 9.4. `KALMAN_Q_BIAS_H` / `KALMAN_R_VISION_H` — Heading

**What it does:** Same as above but for the robot's heading.
**Default:** Q = 0.005, R = 0.01 (radians²)

**Tuning steps:**
1. Heading drift is usually dominated by IMU drift, not odometry
2. If the robot turns 360° and comes back to start but the displayed heading is off by > 10°: increase `KALMAN_Q_BIAS_H` to `0.01`
3. If the displayed heading "jitters" when the robot turns (AprilTag visibility changes): increase `KALMAN_R_VISION_H` to `0.05`

---

## 10. Launch Zone RTP

**Location:** `RobotHardware` — `VECTOR_WEIGHT_DRIVER`

### 10.1. `VECTOR_WEIGHT_DRIVER` — Driver Override Strength

**What it does:** Blending weight for the launch zone pull vs. driver joystick. `0.6` means driver retains 60% authority; higher = more pull toward the nearest polygon zone.
**Default:** 0.6
**Range:** 0.0 – 1.0

**Tuning steps:**
1. Set to `0.3` — driver has strong override; launch zone pull is gentle
2. If drivers ignore the launch zone and shoot from poor positions: increase to `0.7–0.8`
3. If drivers feel like the robot "fights" them near the launch zone: reduce to `0.4`
4. `1.0` = pure driver, no pull. `0.0` = pure pull, no driver input

### 10.2. Zone Vertices (LaunchZoneRTPCommand) — Zone Geometry

The two Marrow `PolygonZone` instances define where the robot is pulled toward:

```
CLOSE_ZONE: (144, 144) — (72, 72) — (0, 144)
FAR_ZONE:   (48, 0)   — (72, 24)  — (96, 0)
```

**When to change:** If the actual DECODE field geometry differs from these coordinates, update the vertex points. The zones are static constants — no runtime tuning needed.

**Tuning tip:** The pull targets the **closest point on the zone boundary**, not the zone center. If a specific shooting position within the zone feels off, trace which polygon edge is nearest and adjust those vertex coordinates accordingly.

### 10.3. Zone Selection Logic

The command picks whichever zone is nearer to the robot (`CLOSE_ZONE` checked first on ties). If the zones overlap, the nearer one always wins. If you want the robot to prefer the far zone in certain conditions, the logic would need to be extended — currently the distance comparison is the sole tiebreaker.

---

## 11. Shoot Sequence

**Location:** `RobotHardware` — `SHOOT_DELAY`

### 11.1. `SHOOT_DELAY` — Gate Open Duration

**What it does:** How long the gate stays open and intake runs during a shot.
**Default:** 2.5 s

**Tuning steps:**
1. Fire a note with a stopwatch — how long does it take from gate-open to gate-close?
2. The note should have cleared the hood before the gate closes
3. If notes jam: increase to 2.8–3.0 s
4. If notes bleed through after the gate closes (double feed): reduce to 2.0–2.2 s

> **5-shot test:** Fire 5 notes consecutively as fast as the state machine allows. All 5 should score; none should jam.

---

## 12. Pedro Pathing

**Location:** `pedroPathing/Constants.java` — `PathConstraints`

```java
public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);
```

`PathConstraints(maxVel, maxAccel, maxJerk, maxCentripetalAccel)` — units are in/sec and in/sec².

### 12.1. `maxVel` — Max Path Speed

**What it does:** Upper speed limit during autonomous path following.
**Default:** 0.99 in/sec

> ⚠️ This default looks suspiciously low (1 in/s). For a typical FTC field (~144 × 144 inches), values of **40–80 in/sec** are normal. Verify this value against the actual units your path builder uses.

**Tuning steps:**
1. Start at `40.0`
2. Run a simple path (e.g., drive 36 inches forward)
3. If the robot is too slow: increase by 10 in/sec
4. If wheels slip on acceleration: reduce by 5 in/sec or increase `maxAccel`

### 12.2. `maxAccel` — Max Acceleration

**What it does:** How quickly the path follower ramps speed up and down.
**Default:** 100 in/sec²

**Tuning steps:**
1. Start at `60.0`
2. If the robot starts paths sluggishly: increase to `80–100`
3. If wheels slip at the start of paths: reduce to `40–50`

### 12.3. `maxJerk` — Max Jerk

**What it does:** Rate of change of acceleration. Higher = snappier starts, more oscillation.
**Default:** 1.0 in/sec³

**Tuning steps:**
1. Leave at `1.0` or `2.0` initially
2. If paths feel jerky at corners: increase to `5.0–10.0`
3. If the robot oscillates around the path: reduce to `0.5`

---

## Tuning Checklist

Print this and check off each stage:

```
□ Flywheel target velocity  — note scores at 30/50/70 inches
□ Flywheel PID (KP/KI)     — settles without oscillation, reaches target in < 1s
□ Flywheel MAX_CURRENT     — reaches target, no overheating
□ Hood hardstops          — servos don't strain at limits
□ Hood LUT                 — scores at all intended distances
□ Hood kH compensation     — no distance change after note loads
□ Gate open/close          — note clears, no bleed-through
□ Intake current limit     — survives jam, runs at full speed
□ Drive MAX_CURRENT        — sufficient torque, no overheating
□ Drive input curve        — smooth low-speed control
□ IMU orientation          — reads correctly after reset
□ Limelight mount angle    — distance matches measured distance
□ Alignment threshold      — locks reliably, no flicker
□ Alignment delay          — feels responsive, not jittery
□ Rotation P gain          — corrects smoothly, no overshoot
□ Kalman Q/R               — pose stable over 30s, vision corrects drift
□ Launch zone weight       — robot pulls to line without fighting driver
□ Shoot delay              — 5 consecutive shots without jam
□ Pedro maxVel/maxAccel    — paths complete in expected time, no wheel slip
```
