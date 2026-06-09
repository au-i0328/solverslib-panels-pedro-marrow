# DECODE Robot Code — Complete Architecture Reference

> Full build-from-scratch reference for the FTC DECODE shoot-assist TeleOp and autonomous.
> Every class, constant, import, and interaction is documented so another AI or engineer
> can reproduce the entire codebase without asking questions.
>
> **Live tuning:** All `@Configurable` constants in `RobotHardware` and `Constants` appear as
> Panels sliders without redeploying. Non-annotated constants are inlined and require a rebuild.

---

## Table of Contents

1. [Prerequisites & Dependencies](#1-prerequisites--dependencies)
2. [Project Structure](#2-project-structure)
3. [Gradle Configuration](#3-gradle-configuration)
4. [File Inventory & Role Summary](#4-file-inventory--role-summary)
5. [RobotHardware — Hardware Declarations & Tuning Constants](#5-robothardware--hardware-declarations--tuning-constants)
6. [RobotState — State Machine Enum](#6-robotstate--state-machine-enum)
7. [DriveSubsystem — Voltage-Based Mecanum Drive](#7-drivesubsystem--voltage-based-mecanum-drive)
8. [FlywheelSubsystem — Custom Voltage-Loop Velocity Control](#8-flywheelsubsystem--custom-voltage-loop-velocity-control)
9. [FlywheelSubsystemSimple — SolversLib Built-In PIDF Alternative](#9-flywheelsubsystemsimple--solverslib-built-in-pidf-alternative)
10. [HoodSubsystem — ServoExGroup Aiming](#10-hoodsubsystem--servoexgroup-aiming)
11. [IntakeSubsystem — Battery-Compensated Intake](#11-intakesubsystem--battery-compensated-intake)
12. [GateSubsystem — Gate Servo Control](#12-gatesubsystem--gate-servo-control)
13. [MasterController — Central State Machine](#13-mastercontroller--central-state-machine)
14. [ShootCommand — Non-Blocking Shoot Sequence](#14-shootcommand--non-blocking-shoot-sequence)
15. [LaunchZoneRTPCommand — Pull-to-Launch-Zone](#15-launchzonertpcommand--pull-to-launch-zone)
16. [BaseZoneRTPCommand — Pull-to-Base-Zone](#16-basezoner tpcommand--pull-to-base-zone)
17. [RunToPointCommand — Pedro RTP Wrapper](#17-runtoPointCommand--pedro-rtp-wrapper)
18. [MainTeleOp — Full TeleOp Loop](#18-mainteleop--full-teleop-loop)
19. [Pedro Pathing Constants & Tuning](#19-pedro-pathing-constants--tuning)
20. [Pedro Pathing Tuning OpModes](#20-pedro-pathing-tuning-opmodes)
21. [Control Flow Diagrams](#21-control-flow-diagrams)
22. [Tuning Constants Quick Reference](#22-tuning-constants-quick-reference)

---

## 1. Prerequisites & Dependencies

### Required Software
- **Android Studio** (Hedgehog or later)
- **FTC Robot Controller SDK** (latest release from FIRST)
- **REV Hardware Client** (for flashing firmware)
- **REV Control Hub** (recommended) or Expansion Hub

### Required Maven Dependencies

| Library | Version | Repository | Purpose |
|---------|---------|------------|---------|
| `org.solverslib:core` | 0.3.4 | `https://maven.brott.dev/` | MotorEx, ServoEx, GamepadEx, command base |
| `org.solverslib:pedroPathing` | 0.3.4 | `https://maven.brott.dev/` | Pedro Pathing integration |
| `com.pedropathing:ftc` | 2.0.6 | `https://maven.brott.dev/` | Deadwheel odometry, Follower |
| `com.pedropathing:telemetry` | 1.0.0 | `https://maven.brott.dev/` | Pedro telemetry |
| `com.skeletonarmyftc.marrow:core` | 1.1.0 | `https://maven.brott.dev/` | PolygonZone, Settings, spatial awareness |
| `com.bylazar:fullpanels` | 1.0.12 | `https://mymaven.bylazar.com/releases` | Live tuning dashboard, field view |
| `com.acmerobotics.dashboard:dashboard` | 0.5.1 | Maven Central | FTC Dashboard (alternative telemetry) |

---

## 2. Project Structure

```
ftc_app/
├── build.gradle                      ← top-level (AGP 8.7.0, Kotlin 2.0.0)
├── build.common.gradle               ← shared SDK configuration
├── build.dependencies.gradle         ← SDK AAR dependency
├── local.properties                 ← sdk.dir=/path/to/Android/sdk
├── settings.gradle
└── TeamCode/
    ├── build.gradle                 ← all dependencies declared here
    └── src/main/java/org/firstinspires/ftc/teamcode/
        ├── RobotHardware.java       ← hardware declarations + tuning constants
        ├── RobotState.java          ← state machine enum
        ├── MainTeleOp.java          ← main TeleOp OpMode
        ├── MasterController.java    ← state machine logic
        ├── subsystems/
        │   ├── DriveSubsystem.java
        │   ├── FlywheelSubsystem.java
        │   ├── FlywheelSubsystemSimple.java
        │   ├── HoodSubsystem.java
        │   ├── IntakeSubsystem.java
        │   └── GateSubsystem.java
        ├── commands/
        │   ├── ShootCommand.java
        │   ├── LaunchZoneRTPCommand.java
        │   ├── BaseZoneRTPCommand.java
        │   └── RunToPointCommand.java
        └── pedroPathing/
            ├── Constants.java       ← Pedro Pathing constants + Follower factory
            └── Tuning.java         ← Pedro tuning OpModes
```

---

## 3. Gradle Configuration

### `build.gradle` (root)

```groovy
buildscript {
    repositories {
        mavenCentral()
        google()
    }
    dependencies {
        classpath 'com.android.tools.build:gradle:8.7.0'
        classpath "org.jetbrains.kotlin:kotlin-gradle-plugin:2.0.0"
    }
}

allprojects {
    repositories {
        mavenCentral()
        google()
        maven { url = "https://mymaven.bylazar.com/releases" }
    }
}
```

### `TeamCode/build.gradle`

```groovy
apply from: '../build.common.gradle'
apply from: '../build.dependencies.gradle'
apply plugin: 'org.jetbrains.kotlin.android'

android {
    namespace = 'org.firstinspires.ftc.teamcode'
    kotlinOptions { jvmTarget = '1.8' }
    compileSdk 34
    defaultConfig { minSdk 24 }
    packagingOptions { jniLibs.useLegacyPackaging true }
}

repositories {
    maven { url = 'https://maven.brott.dev/' }
    maven { url = 'https://mymaven.bylazar.com/releases' }
    maven { url = "https://repo.dairy.foundation/releases" }
    maven { url = "https://repo.dairy.foundation/snapshots" }
}

dependencies {
    implementation project(':FtcRobotController')
    annotationProcessor files('lib/OpModeAnnotationProcessor.jar')

    // SolversLib
    implementation "org.solverslib:core:0.3.4"
    implementation "org.solverslib:pedroPathing:0.3.4"

    // Pedro Pathing
    implementation 'com.pedropathing:ftc:2.0.6'
    implementation 'com.pedropathing:telemetry:1.0.0'

    // Marrow
    implementation 'com.skeletonarmyftc.marrow:core:1.1.0'

    // Panels Dashboard
    implementation "com.bylazar:fullpanels:1.0.12"

    // FTC Dashboard
    implementation "com.acmerobotics.dashboard:dashboard:0.5.1"
}
```

### `local.properties`

```properties
sdk.dir=/Users/your-username/Library/Android/sdk
```

---

## 4. File Inventory & Role Summary

| File | Role |
|------|------|
| `RobotHardware.java` | All hardware declarations, all tuning constants (`public static`), `DriftFilter` Kalman class, `HoodLUT` interpolation, battery voltage, IMU yaw |
| `RobotState.java` | Enum: `INIT`, `INTAKE`, `INTAKE_REVERSE`, `ALIGNING`, `ALIGNED`, `SHOOT` |
| `MainTeleOp.java` | OpMode entry point. Initializes all subsystems, runs the full loop (localization → gamepad → state machine → alignment → RTP → flywheel → hood → drive → rumble → telemetry → pose persistence) |
| `MasterController.java` | State machine logic. Handles all state transitions based on gamepad input and limelight target state |
| `DriveSubsystem.java` | Extends `MecanumDrive`. Fully overrides `driveFieldCentric()`. Per-wheel voltage loop with torque limiting and stall detection |
| `FlywheelSubsystem.java` | Custom per-motor voltage loop: back-EMF compensation + PIDF + torque clamp. Use when `FLYWHEEL_USE_VOLTAGE_LOOP = true` |
| `FlywheelSubsystemSimple.java` | Delegates to `MotorEx.setVelocity()`. Built-in SolversLib PIDF. Use when `FLYWHEEL_USE_VOLTAGE_LOOP = false` |
| `HoodSubsystem.java` | `ServoExGroup` (two mirrored servos). `setPosition()` with hardstop clamp, `setForDistance()` with kH compensation |
| `IntakeSubsystem.java` | Plain `Motor.setPower()` with battery voltage compensation. No encoder |
| `GateSubsystem.java` | Single `ServoEx`. `open()` / `close()` / `isOpen()` |
| `ShootCommand.java` | Non-blocking timed shoot: opens gate + runs intake, waits `SHOOT_DELAY`, closes gate |
| `LaunchZoneRTPCommand.java` | Computes closest point on launch zone polygon, blends pull vector with driver joystick |
| `BaseZoneRTPCommand.java` | Pulls toward alliance base zone center, blends with driver joystick |
| `RunToPointCommand.java` | Thin wrapper around `Follower.follow(Pose)` for auto path running |
| `pedroPathing/Constants.java` | All Pedro constants. `rebuild()` regenerates `TwoWheelLocalizerConstants`, `FollowerConstants`, `PathConstraints`. `createFollower()` factory |
| `pedroPathing/Tuning.java` | Pedro-provided tuning OpModes (Forward, Lateral, Offsets, Velocity, ZPA, Heading, Translational, Drive, Centripetal) |

---

## 5. RobotHardware — Hardware Declarations & Tuning Constants

**Package:** `org.firstinspires.ftc.teamcode`
**File:** `RobotHardware.java`
**Annotation:** `@org.bylazar.ftcontrol.panels.configurables.annotations.Configurable`

### 5.1 Package and Imports

```java
package org.firstinspires.ftc.teamcode;

import static com.seattlesolvers.solverslib.util.MathUtils.clamp;

import androidx.annotation.NonNull;

import com.pedropathing.geometry.Point;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;
import com.seattlesolvers.solverslib.hardware.servos.ServoExGroup;
import com.seattlesolvers.solverslib.hardware.MotorEx;
import com.seattlesolvers.solverslib.hardware.Motor;
import com.seattlesolvers.solverslib.hardware.motors.Motor.GoBILDA;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorEx.CurrentUnit;
import com.qualcomm.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.external.navigation.YawPitchRollAngles;

import java.util.List;
```

### 5.2 Alliance & Field Geometry

```java
public static Alliance ALLIANCE = Alliance.BLUE;

public enum Alliance { RED, BLUE }

/** Goal position in Pedro coordinates (origin = bottom-left, [0,144]).
 *  Switched at runtime to RED_GOAL_COORDS or BLUE_GOAL_COORDS based on alliance. */
public static Point GOAL_COORDS = new Point(144, 72);

public static Point RED_GOAL_COORDS   = new Point(144, 144);
public static Point BLUE_GOAL_COORDS  = new Point(0, 144);
```

### 5.3 Launch Zone Polygons (Marrow PolygonZone)

```java
/** Close launch zone — right-triangle in the scoring corner (Pedro coords). */
public static final com.skeletonarmyftc.marrow.spatial.zone.PolygonZone CLOSE_LAUNCH_ZONE =
        new com.skeletonarmyftc.marrow.spatial.zone.PolygonZone(
                new com.skeletonarmyftc.marrow.spatial.zone.Point(144, 144),
                new com.skeletonarmyftc.marrow.spatial.zone.Point(72,  72),
                new com.skeletonarmyftc.marrow.spatial.zone.Point(0,   144)
        );

/** Far launch zone — smaller triangle toward field center (Pedro coords). */
public static final com.skeletonarmyftc.marrow.spatial.zone.PolygonZone FAR_LAUNCH_ZONE =
        new com.skeletonarmyftc.marrow.spatial.zone.PolygonZone(
                new com.skeletonarmyftc.marrow.spatial.zone.Point(58,  0),
                new com.skeletonarmyftc.marrow.spatial.zone.Point(72,  24),
                new com.skeletonarmyftc.marrow.spatial.zone.Point(96,  0)
        );
```

### 5.4 Base Zone Polygons

```java
/** Blue alliance base zone — 20×20 in² centered at (105.5, 33.5). */
public static final com.skeletonarmyftc.marrow.spatial.zone.PolygonZone BLUE_BASE_ZONE =
        new com.skeletonarmyftc.marrow.spatial.zone.PolygonZone(
                new com.skeletonarmyftc.marrow.spatial.zone.Point(105.5, 33.5),
                20, 20
        );

/** Red alliance base zone — 20×20 in² centered at (38.5, 33.5). */
public static final com.skeletonarmyftc.marrow.spatial.zone.PolygonZone RED_BASE_ZONE =
        new com.skeletonarmyftc.marrow.spatial.zone.PolygonZone(
                new com.skeletonarmyftc.marrow.spatial.zone.Point(38.5, 33.5),
                20, 20
        );

/** Robot footprint size in inches — both width and length (square bot). */
public static double ROBOT_SIZE_INCHES = 18.0;

/** Live robot footprint — position/rotation synced to live pose every loop. */
public static com.skeletonarmyftc.marrow.spatial.zone.PolygonZone ROBOT_ZONE =
        new com.skeletonarmyftc.marrow.spatial.zone.PolygonZone(ROBOT_SIZE_INCHES, ROBOT_SIZE_INCHES);
```

### 5.5 Flywheel Constants

```java
/** Toggle between FlywheelSubsystem (true) and FlywheelSubsystemSimple (false). */
public static final boolean FLYWHEEL_USE_VOLTAGE_LOOP = false;

public static double FLYWHEEL_TARGET_VELOCITY = 2800.0;  // ticks/sec per motor
public static double FLYWHEEL_READY_TOLERANCE = 150.0;   // ticks/sec — isReadyToShoot threshold

// PIDF for left motor
public static double FLYWHEEL_L_KP = 0.0001;
public static double FLYWHEEL_L_KI = 0.001;
public static double FLYWHEEL_L_KD = 0.0;
public static double FLYWHEEL_L_KF = 0.0;

// PIDF for right motor
public static double FLYWHEEL_R_KP = 0.0001;
public static double FLYWHEEL_R_KI = 0.001;
public static double FLYWHEEL_R_KD = 0.0;
public static double FLYWHEEL_R_KF = 0.0;

public static double FLYWHEEL_VELOCITY_OFFSET_JUMP = 50.0; // dpad increment
public static double FLYWHEEL_TPR = 384.5;                  // ticks/revolution (MR encoder)
public static double FLYWHEEL_K_EMF = (12.0 - 0.326) / (6000.0 * 2.0 * Math.PI / 60.0);
public static double FLYWHEEL_MAX_INTEGRAL_VOLTAGE = 3.0;  // volts — I-term ceiling
```

### 5.6 Shoot Sequence

```java
public static double SHOOT_DELAY = 2.5; // seconds — non-blocking via ElapsedTime
```

### 5.7 Gate Servo

```java
public static double GATE_OPEN_POSITION  = 0.55;
public static double GATE_CLOSE_POSITION = 0.0;
```

### 5.8 Hood

```java
public static double HOOD_MIN_POSITION = 0.05;
public static double HOOD_MAX_POSITION = 0.80;
public static double HOOD_ANGLE_OFFSET_JUMP = 0.02;

// Distance → hood position lookup table (linear interpolation)
public static double[] HOOD_DISTANCE_SAMPLES = {
    30.0, 40.0, 50.0, 60.0, 70.0, 80.0
};
public static double[] HOOD_POSITION_SAMPLES = {
    0.75, 0.65, 0.55, 0.45, 0.35, 0.25
};

public static double HOOD_COMPENSATION_COEFFICIENT = 0.0005; // kH factor
```

### 5.9 Limelight

```java
public static int RED_PIPELINE_INDEX   = 0;
public static int BLUE_PIPELINE_INDEX  = 1;
public static double LIMELIGHT_DIST_MIN = 5.0;
public static double LIMELIGHT_DIST_MAX = 120.0;
public static double LIMELIGHT_MOUNT_ANGLE = 25.0;    // degrees from horizontal
public static double LIMELIGHT_DISTANCE_OFFSET = 0.0; // fixed offset
public static double GOAL_HEIGHT = 18.0;               // inches above field floor
```

### 5.10 Alignment

```java
public static double ALIGNMENT_DELAY = 0.15; // seconds — crosshair must stay on target
```

### 5.11 Drive Motor Current Limiting

```java
public static double DRIVE_MAX_CURRENT            = 3.0;  // amps — voltage clamp ceiling
public static double DRIVE_STALL_CURRENT_THRESHOLD = 2.5; // amps — triggers stall reduction
public static int    CURRENT_CHECK_INTERVAL        = 30;  // poll every N loops
public static double DRIVE_INPUT_CURVE_EXP         = 1.5; // joystick exponential curve exponent
```

### 5.12 Vector Addition (Launch Zone RTP)

```java
public static double VECTOR_WEIGHT_DRIVER = 0.7; // 0 = full pull, 1 = full driver override
```

### 5.13 Kalman Localizer Tuning

```java
public static double KALMAN_Q_BIAS_X = 0.02;  // process noise (variance per sqrt(s))
public static double KALMAN_Q_BIAS_Y = 0.02;
public static double KALMAN_Q_BIAS_H = 0.005;
public static double KALMAN_R_VISION_X = 4.0;  // measurement noise variance
public static double KALMAN_R_VISION_Y = 4.0;
public static double KALMAN_R_VISION_H = 0.01;
```

### 5.14 Hardware Fields

```java
// Drive motors
public MotorEx fl, fr, bl, br;

// Flywheel motors
public MotorEx flywheelL, flywheelR;

// Odometry pods — TwoWheelLocalizer
public MotorEx odomPara;    // parallel pod — forward encoder (FL motor)
public MotorEx odomPerpend; // perpendicular pod — lateral encoder (BR motor)

// Intake
public Motor intake;

// Servos
public ServoExGroup hood;   // leader (hoodL) + follower (hoodR, reversed)
public ServoEx gate;

// IMU
public IMU imu;

// Lynx bulk caching
public LynxModule controlHub;

// Voltage sensor
public VoltageSensor voltageSensor;

// Panels dashboard
public org.bylazar.ftcontrol.panels.Panels panels;
public com.seattlesolvers.solverslib.util.TelemetryData telemetryData;

// Live offsets (gamepad2 tuning)
public static double flywheelVelocityOffset = 0.0;
public static double hoodAngleOffset = 0.0;

// Intake voltage
public static double INTAKE_POWER = 10.0;
```

### 5.15 `init()` Method — Hardware Initialization

```java
public void init(@NonNull HardwareMap hwMap) {
    // ── Drive Motors ──────────────────────────────────────────
    fl = new MotorEx(hwMap, "FL", GoBILDA.RPM_435);
    fr = new MotorEx(hwMap, "FR", GoBILDA.RPM_435);
    bl = new MotorEx(hwMap, "BL", GoBILDA.RPM_435);
    br = new MotorEx(hwMap, "BR", GoBILDA.RPM_435);

    fr.setInverted(true);
    br.setInverted(true);

    fl.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
    fr.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
    bl.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
    br.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

    // ── Odometry Pods ─────────────────────────────────────────
    odomPara    = new MotorEx(hwMap, "FL");  // forward pod
    odomPerpend = new MotorEx(hwMap, "BR");  // lateral pod
    odomPara.setInverted(false);
    odomPerpend.setInverted(false);

    // ── Flywheel Motors ──────────────────────────────────────
    flywheelL = new MotorEx(hwMap, "flywheelL");
    flywheelR = new MotorEx(hwMap, "flywheelR");
    flywheelL.setRunMode(Motor.RunMode.VelocityControl);
    flywheelR.setRunMode(Motor.RunMode.VelocityControl);

    // kV = 12 / maxVelocity; kS = 0.15 V overcomes static friction
    double flyKV = 12.0 / (384.5 * 6000.0 / 60.0);
    flywheelL.setFeedforwardCoefficients(0.15, flyKV);
    flywheelR.setFeedforwardCoefficients(0.15, flyKV);

    // ── Intake Motor ──────────────────────────────────────────
    intake = new Motor(hwMap, "intake");
    intake.setInverted(false);

    // ── Hood Servos ───────────────────────────────────────────
    ServoEx _hoodL = new ServoEx(hwMap, "hoodL");
    ServoEx _hoodR = new ServoEx(hwMap, "hoodR");
    _hoodR.setInverted(true);                    // hardware-reversed
    hood = new ServoExGroup(_hoodL, _hoodR);

    // ── Gate Servo ───────────────────────────────────────────
    gate = new ServoEx(hwMap, "gate");
    gate.setPosition(GATE_CLOSE_POSITION);

    // ── IMU ───────────────────────────────────────────────────
    imu = hwMap.get(IMU.class, "imu");
    RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(
            org.firstinspires.ftc.teamcode.pedroPathing.Constants.IMU_LOGO_FACING,
            org.firstinspires.ftc.teamcode.pedroPathing.Constants.IMU_USB_FACING
    );
    imu.initialize(new IMU.Parameters(orientationOnRobot));

    // ── Lynx Bulk Caching ────────────────────────────────────
    List<LynxModule> lynxModules = hwMap.getAll(LynxModule.class);
    for (LynxModule mod : lynxModules) {
        if (mod.isParent()) {
            controlHub = mod;
            break;
        }
    }
    if (controlHub != null) {
        controlHub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
    }

    // ── Voltage Sensor ───────────────────────────────────────
    voltageSensor = hwMap.voltageSensor.get("Control Hub");

    // ── Panels ───────────────────────────────────────────────
    panels = org.bylazar.ftcontrol.panels.Panels.getInstance();
    telemetryData = new com.seattlesolvers.solverslib.util.TelemetryData(
            panels.getTelemetry());
}
```

### 5.16 DriftFilter — Kalman 1D Drift Correction

Three independent single-axis Kalman filters correct pose drift from odometry-vs-vision disagreement.

```java
public static class DriftFilter {
    public double drift = 0.0;
    public double P     = 1.0;
    public final double qBias;
    public final double rVision;

    public DriftFilter(double qBias, double rVision) {
        this.qBias   = qBias;
        this.rVision = rVision;
    }

    /** Predict step — call every loop. Covariance grows as a random walk. */
    public void predict(double dt) {
        P += qBias * qBias * dt;
    }

    /** Update step — call when a fresh MegaTag reading is available. */
    public void update(double odomReading, double visionReading) {
        double measuredDrift = odomReading - visionReading;
        double K = P / (P + rVision);
        drift = drift + K * (measuredDrift - drift);
        P = (1.0 - K) * P;
    }

    /** Apply the learned drift to an odometry reading. */
    public double corrected(double odomReading) {
        return odomReading - drift;
    }
}

// Fields
public DriftFilter filterX, filterY, filterH;

// Init (call once per OpMode)
public void initLocalizer() {
    filterX = new DriftFilter(KALMAN_Q_BIAS_X, KALMAN_R_VISION_X);
    filterY = new DriftFilter(KALMAN_Q_BIAS_Y, KALMAN_R_VISION_Y);
    filterH = new DriftFilter(KALMAN_Q_BIAS_H, KALMAN_R_VISION_H);
}

// Call in loop every tick
public void predictLocalizer(double dt) {
    filterX.predict(dt);
    filterY.predict(dt);
    filterH.predict(dt);
}

// Call when valid MegaTag reading available
public void updateLocalizerFromVision(double odomX, double odomY, double odomH,
                                      double visionX, double visionY, double visionH) {
    filterX.update(odomX, visionX);
    filterY.update(odomY, visionY);
    filterH.update(odomH, visionH);
}

// Pose getters
public double getCorrectedX(double odomX) { return filterX.corrected(odomX); }
public double getCorrectedY(double odomY) { return filterY.corrected(odomY); }
public double getCorrectedH(double odomH) { return filterH.corrected(odomH); }
```

### 5.17 HoodLUT — Linear Interpolation Lookup Table

```java
public static class HoodLUT {
    private final double[] dist;
    private final double[] pos;

    public HoodLUT(double[] distSamples, double[] posSamples) {
        this.dist = distSamples;
        this.pos  = posSamples;
    }

    public double get(double distance) {
        if (distance <= dist[0]) return pos[0];
        if (distance >= dist[dist.length - 1]) return pos[pos.length - 1];
        for (int i = 0; i < dist.length - 1; i++) {
            if (distance >= dist[i] && distance <= dist[i + 1]) {
                double t = (distance - dist[i]) / (dist[i + 1] - dist[i]);
                return pos[i] + t * (pos[i + 1] - pos[i]);
            }
        }
        return pos[pos.length - 1];
    }
}

public static final HoodLUT HOOD_LUT =
        new HoodLUT(HOOD_DISTANCE_SAMPLES, HOOD_POSITION_SAMPLES);

public static double hoodPosition(double distance, double offset) {
    double raw = HOOD_LUT.get(distance) + offset;
    return clamp(raw, HOOD_MIN_POSITION, HOOD_MAX_POSITION);
}
```

### 5.18 Utility Methods

```java
public double batteryVoltage() {
    return voltageSensor.getVoltage();
}

public double getYawRadians() {
    return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
}

/** Exponential input curve. exp=1.0 is linear; >1.0 gives more sensitivity near zero. */
public static double applyInputCurve(double input, double exp) {
    return Math.copySign(Math.pow(Math.abs(input), exp), input);
}

// Alliance selection
public static Alliance selectAlliance(boolean triangle, boolean circle) {
    if (triangle) return Alliance.RED;
    if (circle)   return Alliance.BLUE;
    return ALLIANCE;
}

public static Point goalCoordsForAlliance(Alliance a) {
    return (a == Alliance.RED) ? RED_GOAL_COORDS : BLUE_GOAL_COORDS;
}

// Gamepad2 offset helpers
public void adjustFlywheelOffset(boolean dpadUp, boolean dpadDown) {
    if (dpadUp)   flywheelVelocityOffset += FLYWHEEL_VELOCITY_OFFSET_JUMP;
    if (dpadDown) flywheelVelocityOffset -= FLYWHEEL_VELOCITY_OFFSET_JUMP;
}

public void adjustHoodOffset(boolean dpadLeft, boolean dpadRight) {
    if (dpadLeft)  hoodAngleOffset += HOOD_ANGLE_OFFSET_JUMP;
    if (dpadRight) hoodAngleOffset -= HOOD_ANGLE_OFFSET_JUMP;
}

// Stall detection
private int stallCheckCounter = 0;
private boolean driveStalling = false;

public boolean isDriveStallingAny() {
    stallCheckCounter++;
    boolean anyStalling =
        fl.motor.getCurrent(CurrentUnit.AMPS) > DRIVE_STALL_CURRENT_THRESHOLD ||
        fr.motor.getCurrent(CurrentUnit.AMPS) > DRIVE_STALL_CURRENT_THRESHOLD ||
        bl.motor.getCurrent(CurrentUnit.AMPS) > DRIVE_STALL_CURRENT_THRESHOLD ||
        br.motor.getCurrent(CurrentUnit.AMPS) > DRIVE_STALL_CURRENT_THRESHOLD;

    if (anyStalling) {
        driveStalling = true;
        stallCheckCounter = 0;
    } else if (stallCheckCounter >= CURRENT_CHECK_INTERVAL) {
        driveStalling = false;
        stallCheckCounter = 0;
    }
    return driveStalling;
}

// Bulk cache clearing
public void clearBulkCache() {
    if (controlHub != null) controlHub.clearBulkCache();
}
```

---

## 6. RobotState — State Machine Enum

**File:** `RobotState.java`

```java
package org.firstinspires.ftc.teamcode;

public enum RobotState {
    INIT,
    INTAKE,
    INTAKE_REVERSE,
    ALIGNING,
    ALIGNED,
    SHOOT
}
```

---

## 7. DriveSubsystem — Voltage-Based Mecanum Drive

**Package:** `org.firstinspires.ftc.teamcode.subsystems`
**Parent:** `SolversLib MecanumDrive`
**File:** `DriveSubsystem.java`

### Physics Constants (GoBilda 435 RPM)

| Constant | Value | Derivation |
|----------|-------|------------|
| `TPR` | 384.5 ticks/rev | encoder CPR |
| `I_STALL` | 9.2 A | motor spec |
| `R_MOTOR` | 1.30 Ω | 12 V / 9.2 A |
| `V_RESISTOR` | 0.326 V | 0.25 A × R |
| `OMEGA_NOLOAD` | 45.55 rad/s | 435 RPM × 2π/60 |
| `K_EMF` | 0.256 V/(rad/s) | (12 − V_RESISTOR) / OMEGA_NOLOAD |

### Full Code

```java
package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.follower.Follower;
import com.seattlesolvers.solverslib.drivebase.MecanumDrive;
import com.seattlesolvers.solverslib.geometry.Vector2d;
import com.seattlesolvers.solverslib.hardware.MotorEx;
import com.qualcomm.robotcore.hardware.DcMotorEx.CurrentUnit;

public class DriveSubsystem extends MecanumDrive {

    // Motor physics (GoBilda 435 RPM — 13.7:1 planetary, 384.5 CPR)
    private static final double TPR         = 384.5;
    private static final double I_STALL    = 9.2;
    private static final double R_MOTOR    = 12.0 / I_STALL;                          // ≈ 1.30 Ω
    private static final double V_RESISTOR  = 0.25 * R_MOTOR;                          // ≈ 0.326 V
    private static final double OMEGA_NOLOAD = 435.0 * 2.0 * Math.PI / 60.0;          // ≈ 45.55 rad/s
    private static final double K_EMF = (12.0 - V_RESISTOR) / OMEGA_NOLOAD;            // ≈ 0.256 V/(rad/s)

    private static final double MAX_CURRENT = RobotHardware.DRIVE_MAX_CURRENT;  // 3.0 A

    // Stall detection
    public static int STALL_CHECK_INTERVAL = 10;
    private int   stallCounter = 0;
    private boolean stalling   = false;
    private double stallPowerScale = 1.0;
    private static final double STALL_POWER_FLOOR = 0.3;

    private final MotorEx[] allMotors;

    public DriveSubsystem(RobotHardware hw, Follower follower) {
        super(true, hw.fl, hw.fr, hw.bl, hw.br);
        allMotors = new MotorEx[]{ hw.fl, hw.fr, hw.bl, hw.br };

        // kV = 12 V / maxVelocity → set(1.0) = max speed
        double maxVel = TPR * 435.0 / 60.0;  // ≈ 2769 ticks/s
        double kV = 12.0 / maxVel;
        double kS = 0.15;
        for (MotorEx m : allMotors) {
            m.setFeedforwardCoefficients(kS, kV);
        }
    }

    /**
     * Full override of MecanumDrive field-centric drive.
     * Per-wheel voltage loop:
     *   vTarget   = frac × 12 V
     *   vBackEmf  = K_EMF × ω  (opposes applied voltage)
     *   iTarget   = |frac| < 0.01 → 0 (coast); else → MAX_CURRENT
     *   vMin       = vBackEmf − iTarget × R_MOTOR
     *   vMax       = vBackEmf + iTarget × R_MOTOR
     *   vClamped   = clamp(vTarget, vMin, vMax)
     *   power      = vClamped / batteryVoltage
     *   if stalling: power *= stallPowerScale
     */
    @Override
    public void driveFieldCentric(double strafe, double forward, double turn, double gyroAngle) {
        double exp = RobotHardware.DRIVE_INPUT_CURVE_EXP;
        strafe  = RobotHardware.applyInputCurve(strafe,  exp);
        forward = RobotHardware.applyInputCurve(forward, exp);
        turn    = RobotHardware.applyInputCurve(turn,    exp);

        strafe  = clipRange(strafe);
        forward = clipRange(forward);
        turn    = clipRange(turn);

        // Field-centric rotation
        Vector2d input = new Vector2d(strafe, forward);
        input = input.rotateBy(-gyroAngle);

        double theta = input.angle();

        double[] raw = new double[4];
        raw[kFrontLeft]  = Math.sin(theta + Math.PI / 4);
        raw[kFrontRight] = Math.sin(theta - Math.PI / 4);
        raw[kBackLeft]   = Math.sin(theta - Math.PI / 4);
        raw[kBackRight]  = Math.sin(theta + Math.PI / 4);

        normalize(raw, input.magnitude());

        raw[kFrontLeft]  += turn;
        raw[kFrontRight] -= turn;
        raw[kBackLeft]   += turn;
        raw[kBackRight]  -= turn;

        normalize(raw);

        applyVoltageControl(
            raw[kFrontLeft],
            raw[kFrontRight],
            raw[kBackLeft],
            raw[kBackRight]
        );
    }

    private void applyVoltageControl(double flPow, double frPow, double blPow, double brPow) {
        double batt = RobotHardware.batteryVoltage();
        double[] fractions = { flPow, frPow, blPow, brPow };

        stallCounter++;
        boolean anyStalling = false;

        for (int i = 0; i < 4; i++) {
            double frac = fractions[i];
            MotorEx m   = allMotors[i];

            double vTarget  = frac * 12.0;
            double omega    = m.getVelocity() / TPR * 2.0 * Math.PI;  // rad/s
            double vBackEmf = K_EMF * omega;

            double iTarget = Math.abs(frac) < 0.01 ? 0.0 : MAX_CURRENT;

            double vMin = vBackEmf - iTarget * R_MOTOR;
            double vMax = vBackEmf + iTarget * R_MOTOR;
            double vClamped = Math.max(vMin, Math.min(vMax, vTarget));
            double power    = vClamped / batt;

            if (stallPowerScale < 1.0) {
                power *= stallPowerScale;
            }

            m.set(power);

            if (m.getCurrent(CurrentUnit.AMPS) > MAX_CURRENT) {
                anyStalling = true;
            }
        }

        if (anyStalling) {
            stallCounter = 0;
            stalling = true;
            stallPowerScale = Math.max(stallPowerScale - 0.15, STALL_POWER_FLOOR);
        } else if (STALL_CHECK_INTERVAL > 0 && stallCounter >= STALL_CHECK_INTERVAL) {
            stalling = false;
            stallCounter = 0;
            stallPowerScale = Math.min(stallPowerScale + 0.10, 1.0);
        }
    }

    public boolean isStalling() { return stalling; }

    @Override
    public void stop() {
        for (MotorEx m : allMotors) m.stopMotor();
        stalling = false;
        stallCounter = 0;
        stallPowerScale = 1.0;
    }
}
```

---

## 8. FlywheelSubsystem — Custom Voltage-Loop Velocity Control

**Package:** `org.firstinspires.ftc.teamcode.subsystems`
**Parent:** `SolversLib Subsystem`
**File:** `FlywheelSubsystem.java`

Use this when `RobotHardware.FLYWHEEL_USE_VOLTAGE_LOOP = true`.

### Voltage Loop Per Motor (per cycle)

```
ω_measured  = motor.getVelocity() / TPR × 2π        (rad/s)
vBackEmf   = K_EMF × ω_measured                    (opposes applied voltage)

err        = targetVelocity − motor.getVelocity()    (ticks/s)
integral   = clamp(integral + err·dt, −MAX_INT/kI, MAX_INT/kI)  (anti-windup)
derivative = (dt > 0.0001) ? (err − prevErr) / dt : 0
vPID      = kP·err + kI·integral + kD·derivative + kF·targetVelocity

vTarget    = vPID + vBackEmf                        (back-EMF compensation)
power      = vTarget / batteryVoltage
```

### Full Code

```java
package org.firstinspires.ftc.teamcode.subsystems;

import com.seattlesolvers.solverslib.hardware.MotorEx;

public class FlywheelSubsystem extends com.seattlesolvers.solverslib.command.Subsystem {
    private final MotorEx motorL;
    private final MotorEx motorR;

    public double targetVelocity = RobotHardware.FLYWHEEL_TARGET_VELOCITY;

    // Per-motor PIDF state
    private double integralL = 0.0;
    private double integralR = 0.0;
    private double prevErrL  = 0.0;
    private double prevErrR  = 0.0;

    public FlywheelSubsystem(RobotHardware hw) {
        this.motorL = hw.flywheelL;
        this.motorR = hw.flywheelR;
    }

    public void addOffset(double delta) {
        targetVelocity += delta;
    }

    public double getTargetVelocity() { return targetVelocity; }
    public double getVelocityL()      { return motorL.getVelocity(); }
    public double getVelocityR()      { return motorR.getVelocity(); }

    public boolean isAtTarget() {
        double tol = RobotHardware.FLYWHEEL_READY_TOLERANCE;
        return Math.abs(getVelocityL() - targetVelocity) < tol
            && Math.abs(getVelocityR() - targetVelocity) < tol;
    }

    public void update(double dt) {
        double batt = RobotHardware.batteryVoltage();

        applyVoltageLoop(motorL, targetVelocity, dt, batt,
                RobotHardware.FLYWHEEL_TPR,
                RobotHardware.FLYWHEEL_K_EMF,
                RobotHardware.FLYWHEEL_L_KP,
                RobotHardware.FLYWHEEL_L_KI,
                RobotHardware.FLYWHEEL_L_KD,
                RobotHardware.FLYWHEEL_L_KF);

        applyVoltageLoop(motorR, targetVelocity, dt, batt,
                RobotHardware.FLYWHEEL_TPR,
                RobotHardware.FLYWHEEL_K_EMF,
                RobotHardware.FLYWHEEL_R_KP,
                RobotHardware.FLYWHEEL_R_KI,
                RobotHardware.FLYWHEEL_R_KD,
                RobotHardware.FLYWHEEL_R_KF);
    }

    private void applyVoltageLoop(
            MotorEx motor,
            double targetVel,
            double dt,
            double batt,
            double tpr,
            double kEmf,
            double kp, double ki, double kd, double kf
    ) {
        double omegaMeas = motor.getVelocity() / tpr * 2.0 * Math.PI;
        double vBackEmf  = kEmf * omegaMeas;

        double err = targetVel - motor.getVelocity();

        double integral = (motor == motorL) ? integralL : integralR;
        double prevErr  = (motor == motorL) ? prevErrL  : prevErrR;

        // Bound I-term by voltage contribution so the clamp is ki-independent
        double maxIntSum = (ki == 0.0) ? 0.0
                : (RobotHardware.FLYWHEEL_MAX_INTEGRAL_VOLTAGE / ki);
        integral = clamp(integral + err * dt, -maxIntSum, maxIntSum);

        double deriv = (dt > 0.0001) ? (err - prevErr) / dt : 0.0;

        double vPID    = kp * err + ki * integral + kd * deriv + kf * targetVel;
        double vTarget = vPID + vBackEmf;

        motor.set(vTarget / batt);

        if (motor == motorL) {
            integralL = integral; prevErrL = err;
        } else {
            integralR = integral; prevErrR = err;
        }
    }

    @Deprecated
    public void update() { update(0.001); }

    public void reset() {
        integralL = 0.0; integralR = 0.0;
        prevErrL  = 0.0; prevErrR  = 0.0;
    }

    public void stop() {
        motorL.stopMotor();
        motorR.stopMotor();
        reset();
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}
```

---

## 9. FlywheelSubsystemSimple — SolversLib Built-In PIDF Alternative

**Package:** `org.firstinspires.ftc.teamcode.subsystems`
**Parent:** `SolversLib Subsystem`
**File:** `FlywheelSubsystemSimple.java`

Use this when `RobotHardware.FLYWHEEL_USE_VOLTAGE_LOOP = false`.

```java
package org.firstinspires.ftc.teamcode.subsystems;

import com.seattlesolvers.solverslib.hardware.MotorEx;

public class FlywheelSubsystemSimple extends com.seattlesolvers.solverslib.command.Subsystem {
    private final MotorEx motorL;
    private final MotorEx motorR;

    public double targetVelocity = RobotHardware.FLYWHEEL_TARGET_VELOCITY;

    public FlywheelSubsystemSimple(RobotHardware hw) {
        this.motorL = hw.flywheelL;
        this.motorR = hw.flywheelR;

        motorL.setVelocityCoefficients(
                RobotHardware.FLYWHEEL_L_KP,
                RobotHardware.FLYWHEEL_L_KI,
                RobotHardware.FLYWHEEL_L_KD,
                RobotHardware.FLYWHEEL_L_KF);

        motorR.setVelocityCoefficients(
                RobotHardware.FLYWHEEL_R_KP,
                RobotHardware.FLYWHEEL_R_KI,
                RobotHardware.FLYWHEEL_R_KD,
                RobotHardware.FLYWHEEL_R_KF);
    }

    public void addOffset(double delta) {
        targetVelocity += delta;
    }

    public double getTargetVelocity() { return targetVelocity; }
    public double getVelocityL()      { return motorL.getVelocity(); }
    public double getVelocityR()      { return motorR.getVelocity(); }

    public boolean isAtTarget() {
        double tol = RobotHardware.FLYWHEEL_READY_TOLERANCE;
        return Math.abs(getVelocityL() - targetVelocity) < tol
            && Math.abs(getVelocityR() - targetVelocity) < tol;
    }

    public void update() {
        motorL.setVelocity(targetVelocity);
        motorR.setVelocity(targetVelocity);
    }

    @Deprecated
    public void update(double dt) { update(); }

    public void stop() {
        motorL.stopMotor();
        motorR.stopMotor();
    }
}
```

---

## 10. HoodSubsystem — ServoExGroup Aiming

**Package:** `org.firstinspires.ftc.teamcode.subsystems`
**Parent:** `SolversLib Subsystem`
**File:** `HoodSubsystem.java`

### Control Modes

| Method | Purpose |
|--------|---------|
| `setPosition(double p)` | Clamped direct position write |
| `setForDistance(double d, double offset, double actualVel, double targetVel)` | Full aiming: LUT + offset + kH compensation |
| `setRaw(double p)` | Unclamped — for calibration only |

```java
package org.firstinspires.ftc.teamcode.subsystems;

import static com.seattlesolvers.solverslib.util.MathUtils.clamp;

import com.seattlesolvers.solverslib.hardware.servos.ServoExGroup;
import org.firstinspires.ftc.teamcode.RobotHardware;

public class HoodSubsystem extends com.seattlesolvers.solverslib.command.Subsystem {
    private final ServoExGroup hood;

    public HoodSubsystem(RobotHardware hw) {
        this.hood = hw.hood;
    }

    public void setPosition(double position) {
        double p = clamp(position,
                RobotHardware.HOOD_MIN_POSITION,
                RobotHardware.HOOD_MAX_POSITION);
        hood.setPosition(p);
    }

    /**
     * Aiming with LUT + offset + kH velocity drop compensation.
     * @param distance       inches to goal (from Limelight)
     * @param offset         gamepad2 dpad tuning offset
     * @param actualVelocity current flywheel velocity (ticks/s)
     * @param targetVelocity flywheel target velocity (ticks/s)
     */
    public void setForDistance(double distance, double offset,
                               double actualVelocity, double targetVelocity) {
        double basePos = RobotHardware.hoodPosition(distance, offset);
        double velDrop    = targetVelocity - actualVelocity;
        double hoodOffset = velDrop * RobotHardware.HOOD_COMPENSATION_COEFFICIENT;
        setPosition(basePos + hoodOffset);
    }

    /** Unclamped passthrough — for calibration only. */
    public void setRaw(double rawPosition) {
        hood.setPosition(rawPosition);
    }

    public double getPosition() {
        return hood.get();
    }
}
```

---

## 11. IntakeSubsystem — Battery-Compensated Intake

**Package:** `org.firstinspires.ftc.teamcode.subsystems`
**Parent:** `SolversLib Subsystem`
**File:** `IntakeSubsystem.java`

```java
package org.firstinspires.ftc.teamcode.subsystems;

import com.seattlesolvers.solverslib.hardware.Motor;
import org.firstinspires.ftc.teamcode.RobotHardware;

public class IntakeSubsystem extends com.seattlesolvers.solverslib.command.Subsystem {
    private final Motor motor;

    public IntakeSubsystem(RobotHardware hw) {
        this.motor = hw.intake;
    }

    public void runForward() {
        motor.set(RobotHardware.INTAKE_POWER / RobotHardware.batteryVoltage());
    }

    public void runReverse() {
        motor.set(-RobotHardware.INTAKE_POWER / RobotHardware.batteryVoltage());
    }

    public void stop() {
        motor.stopMotor();
    }

    public boolean isRunning() {
        return Math.abs(motor.get()) > 0.01;
    }
}
```

---

## 12. GateSubsystem — Gate Servo Control

**Package:** `org.firstinspires.ftc.teamcode.subsystems`
**Parent:** `SolversLib Subsystem`
**File:** `GateSubsystem.java`

```java
package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.RobotHardware;

public class GateSubsystem extends com.seattlesolvers.solverslib.command.Subsystem {
    private final Servo gate;

    public GateSubsystem(RobotHardware hw) {
        this.gate = hw.gate;
        gate.setPosition(RobotHardware.GATE_CLOSE_POSITION);
    }

    public void open() {
        gate.setPosition(RobotHardware.GATE_OPEN_POSITION);
    }

    public void close() {
        gate.setPosition(RobotHardware.GATE_CLOSE_POSITION);
    }

    public boolean isOpen() {
        return gate.getPosition()
            > (RobotHardware.GATE_OPEN_POSITION + RobotHardware.GATE_CLOSE_POSITION) / 2.0;
    }

    public double getPosition() {
        return gate.getPosition();
    }
}
```

---

## 13. MasterController — Central State Machine

**Package:** `org.firstinspires.ftc.teamcode`
**File:** `MasterController.java`

### State Transition Diagram

```
INIT ──► INTAKE ───────────────────────────► ALIGNING ──► ALIGNED
          │    ▲                            ▲     │          │
          │    │                            │     │          │
     left bumper                           │     │    left trigger + ready
          │    │                            │     │    ──► SHOOT ──► INTAKE
          ▼    │                      right trigger│     │
   INTAKE_REVERSE                        released       ▼
                                                     INTAKE
```

### Priority Rules
1. **Circle (gamepad2)** → instant `INTAKE`, overrides everything
2. **Left bumper** → `INTAKE_REVERSE`, overrides everything
3. **Right bumper (gamepad1)** → `INTAKE`
4. **Right trigger** → `ALIGNING` / maintain `ALIGNED`
5. **Left trigger + ready** → `SHOOT`
6. During `SHOOT` — gamepads are fully neglected

### Callbacks (injected by OpMode)

```java
// Called by OpMode to feed limelight data without hardware coupling
public interface LimelightProvider {
    LLResult get();          // latest Limelight result
    double getAngleToGoal(); // tx in degrees
    boolean hasTarget();     // isValid()
}

// Called by OpMode to get Kalman-corrected pose
public interface PoseProvider {
    double getX();
    double getY();
    double getH();           // radians
    Point getGoalCoords();
}

// Called by OpMode to check flywheel + zone readiness
public interface ReadyCheck {
    boolean isReady();
}

// Called when shoot timer completes
public interface ShootDone {
    void onShootDone();
}
```

### Full Code

```java
package org.firstinspires.ftc.teamcode;

import com.pedropathing.geometry.Point;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.commands.ShootCommand;
import org.firstinspires.ftc.teamcode.subsystems.FlywheelSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.GateSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

public class MasterController {

    private RobotState state = RobotState.INIT;
    private boolean wasShooting = false;

    private final FlywheelSubsystem flywheel;
    private final GateSubsystem gate;
    private final IntakeSubsystem intake;
    private final ShootCommand shootCommand;

    private ElapsedTime alignmentTimer = new ElapsedTime();
    private boolean aligningActive = false;

    // Callbacks
    public interface ReadyCheck    { boolean isReady(); }
    public interface ShootDone     { void onShootDone(); }
    public interface LimelightProvider {
        com.qualcomm.hardware.limelightvision.LLResult get();
        double getAngleToGoal();
        boolean hasTarget();
    }
    public interface PoseProvider {
        double getX();
        double getY();
        double getH();
        Point getGoalCoords();
    }

    private ReadyCheck readyCheck;
    private ShootDone shootDone;
    private LimelightProvider limelightProvider;
    private PoseProvider poseProvider;

    // Edge detection state
    private boolean prevLeftTrigger = false;
    private boolean prevShare = false;

    public MasterController(FlywheelSubsystem flywheel, GateSubsystem gate, IntakeSubsystem intake) {
        this.flywheel = flywheel;
        this.gate = gate;
        this.intake = intake;
        this.shootCommand = new ShootCommand(flywheel, gate, intake, () -> {
            state = RobotState.INTAKE;
            if (shootDone != null) shootDone.onShootDone();
        });
    }

    public void setReadyCheck(ReadyCheck check)            { this.readyCheck = check; }
    public void setShootDoneCallback(ShootDone done)        { this.shootDone = done; }
    public void setLimelightProvider(LimelightProvider l)  { this.limelightProvider = l; }
    public void setPoseProvider(PoseProvider p)            { this.poseProvider = p; }
    public RobotState getState()                           { return state; }
    public void setState(RobotState s)                      { this.state = s; }

    /**
     * Main update — call every loop.
     * @param gamepad1  driver gamepad
     * @param gamepad2  operator gamepad
     * @param dt        loop time in seconds
     */
    public void update(Gamepad gamepad1, Gamepad gamepad2, double dt) {
        // ── SHOOT TIMER: gamepads neglected ──────────────────────────
        if (wasShooting) {
            shootCommand.execute();
            if (shootCommand.isFinished()) {
                shootCommand.end();
                wasShooting = false;
            }
            return;
        }

        // ── EDGE DETECTION ──────────────────────────────────────────
        boolean leftBumper   = gamepad1.left_bumper || gamepad2.left_bumper;
        boolean rightTrigger = gamepad1.right_trigger > 0.5;

        boolean leftTriggerRising = (gamepad1.left_trigger > 0.5) && !prevLeftTrigger;
        prevLeftTrigger = gamepad1.left_trigger > 0.5;

        boolean shareRising = gamepad2.share && !prevShare;
        prevShare = gamepad2.share;

        // ── PRIORITY 1: Circle → INTAKE ──────────────────────────────
        if (gamepad2.circle) {
            state = RobotState.INTAKE;
            gate.close();
            intake.stop();
            return;
        }

        // ── PRIORITY 2: Left bumper → INTAKE_REVERSE ─────────────────
        if (leftBumper) {
            state = RobotState.INTAKE_REVERSE;
            gate.close();
            intake.runReverse();
            return;
        }

        // ── PRIORITY 3: Right bumper → INTAKE ────────────────────────
        if (gamepad1.right_bumper) {
            state = RobotState.INTAKE;
            gate.close();
            intake.stop();
            return;
        }

        // ── PER-STATE LOGIC ──────────────────────────────────────────
        switch (state) {
            case INIT:
            case INTAKE:
                intake.runForward();
                gate.close();
                if (rightTrigger) {
                    state = RobotState.ALIGNING;
                    alignmentTimer.reset();
                    aligningActive = false;
                }
                break;

            case INTAKE_REVERSE:
                intake.runReverse();
                gate.close();
                break;

            case ALIGNING: {
                if (!rightTrigger) {
                    state = RobotState.INTAKE;
                    alignmentTimer.reset();
                    aligningActive = false;
                    break;
                }

                if (limelightProvider != null && limelightProvider.hasTarget()) {
                    double tx = Math.abs(limelightProvider.getAngleToGoal());
                    if (tx < 2.0) {                          // within 2° of crosshair
                        aligningActive = true;
                    } else {
                        alignmentTimer.reset();
                        aligningActive = false;
                    }

                    if (aligningActive && alignmentTimer.seconds() >= RobotHardware.ALIGNMENT_DELAY) {
                        state = RobotState.ALIGNED;
                        alignmentTimer.reset();
                        aligningActive = false;
                    }
                } else {
                    alignmentTimer.reset();
                    aligningActive = false;
                }
                break;
            }

            case ALIGNED: {
                if (!rightTrigger) {
                    state = RobotState.INTAKE;
                    alignmentTimer.reset();
                    break;
                }

                if (limelightProvider != null && !limelightProvider.hasTarget()) {
                    state = RobotState.ALIGNING;
                    alignmentTimer.reset();
                    aligningActive = false;
                    break;
                }

                // Left trigger edge-triggered fires shot
                if (leftTriggerRising) {
                    if (gamepad2.share || (readyCheck != null && readyCheck.isReady())) {
                        state = RobotState.SHOOT;
                        wasShooting = true;
                        shootCommand.execute();
                    }
                }
                break;
            }

            case SHOOT:
                // Handled at top of loop
                break;
        }
    }

    public void reset() {
        state = RobotState.INIT;
        wasShooting = false;
        prevLeftTrigger = false;
        prevShare = false;
        alignmentTimer.reset();
        aligningActive = false;
    }
}
```

---

## 14. ShootCommand — Non-Blocking Shoot Sequence

**Package:** `org.firstinspires.ftc.teamcode.commands`
**File:** `ShootCommand.java`

Implements `com.seattlesolvers.solverslib.command.Command`.

### Timing

| Time | Action |
|------|--------|
| t = 0 | `gate.open()`, `intake.runForward()` |
| t ≥ `SHOOT_DELAY` | `gate.close()`, `intake.stop()`, callback runs |

```java
package org.firstinspires.ftc.teamcode.commands;

import com.seattlesolvers.solverslib.command.Command;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.FlywheelSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.GateSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

public class ShootCommand implements Command {
    private final FlywheelSubsystem flywheel;
    private final GateSubsystem gate;
    private final IntakeSubsystem intake;
    private final Runnable onComplete;

    private final com.qualcomm.robotcore.util.ElapsedTime timer =
        new com.qualcomm.robotcore.util.ElapsedTime();
    private boolean started = false;

    public ShootCommand(FlywheelSubsystem flywheel, GateSubsystem gate,
                        IntakeSubsystem intake, Runnable onComplete) {
        this.flywheel = flywheel;
        this.gate = gate;
        this.intake = intake;
        this.onComplete = onComplete;
    }

    @Override
    public void execute() {
        if (!started) {
            started = true;
            timer.reset();
            gate.open();
            intake.runForward();
            return;
        }

        intake.runForward();

        if (timer.seconds() >= RobotHardware.SHOOT_DELAY) {
            gate.close();
            intake.stop();
            if (onComplete != null) onComplete.run();
        }
    }

    @Override
    public boolean isFinished() {
        return started && timer.seconds() >= RobotHardware.SHOOT_DELAY;
    }

    @Override
    public void end() {
        gate.close();
        intake.stop();
    }
}
```

---

## 15. LaunchZoneRTPCommand — Pull-to-Launch-Zone

**Package:** `org.firstinspires.ftc.teamcode.commands`
**File:** `LaunchZoneRTPCommand.java`

Implements `com.seattlesolvers.solverslib.command.Command`.

### Algorithm

```
1. On activation: sync ROBOT_ZONE to live pose, find nearest zone
2. Every loop:
   a. If driver input magnitude increased > 5% → update target to new nearest boundary point
   b. Compute pull unit vector toward live target point
   c. Rotate driver joystick to world frame (rotate by robotH)
   d. Blend: blended = pull × (1−w) + driverWorld × w
   e. Normalize to unit magnitude
   f. Write to drive inputs
```

### Constructor Parameters

| Parameter | Type | Description |
|-----------|------|-------------|
| `getRobotX/Y/H` | `Supplier<Double>` | Kalman-corrected pose |
| `getDriverFwd/Strafe` | `Supplier<Double>` | Raw gamepad joystick values |
| `setBlendedFwd/Strafe` | `DoubleConsumer` | Write blended drive inputs |
| `getBlendedFwd/Strafe` | `Supplier<Double>` | Previous frame's blended output |
| `onTargetPointChanged` | `DoubleConsumer` | Optional hook when target updates |
| `getRobotZone` | `Supplier<PolygonZone>` | Live robot footprint zone |

```java
package org.firstinspires.ftc.teamcode.commands;

import com.seattlesolvers.solverslib.command.Command;
import com.skeletonarmyftc.marrow.spatial.zone.PolygonZone;
import com.skeletonarmyftc.marrow.spatial.zone.Point;
import org.firstinspires.ftc.teamcode.RobotHardware;

public class LaunchZoneRTPCommand implements Command {

    private final java.util.function.Supplier<Double> getRobotX;
    private final java.util.function.Supplier<Double> getRobotY;
    private final java.util.function.Supplier<Double> getRobotH;
    private final java.util.function.Supplier<Double> getDriverFwd;
    private final java.util.function.Supplier<Double> getDriverStrafe;
    private final java.util.function.DoubleConsumer setBlendedFwd;
    private final java.util.function.DoubleConsumer setBlendedStrafe;
    private final java.util.function.Supplier<Double> getBlendedFwd;
    private final java.util.function.Supplier<Double> getBlendedStrafe;
    private final java.util.function.DoubleConsumer onTargetPointChanged;
    private final java.util.function.Supplier<PolygonZone> getRobotZone;

    private boolean active = false;
    private double targetX;
    private double targetY;

    public LaunchZoneRTPCommand(
            java.util.function.Supplier<Double> getRobotX,
            java.util.function.Supplier<Double> getRobotY,
            java.util.function.Supplier<Double> getRobotH,
            java.util.function.Supplier<Double> getDriverFwd,
            java.util.function.Supplier<Double> getDriverStrafe,
            java.util.function.DoubleConsumer setBlendedFwd,
            java.util.function.DoubleConsumer setBlendedStrafe,
            java.util.function.Supplier<Double> getBlendedFwd,
            java.util.function.Supplier<Double> getBlendedStrafe,
            java.util.function.DoubleConsumer onTargetPointChanged,
            java.util.function.Supplier<PolygonZone> getRobotZone) {
        this.getRobotX = getRobotX;
        this.getRobotY = getRobotY;
        this.getRobotH = getRobotH;
        this.getDriverFwd = getDriverFwd;
        this.getDriverStrafe = getDriverStrafe;
        this.setBlendedFwd = setBlendedFwd;
        this.setBlendedStrafe = setBlendedStrafe;
        this.getBlendedFwd = getBlendedFwd;
        this.getBlendedStrafe = getBlendedStrafe;
        this.onTargetPointChanged = onTargetPointChanged;
        this.getRobotZone = getRobotZone;
    }

    public void setActive(boolean a) {
        if (a && !this.active) {
            // Fresh activation — sync robot footprint, find nearest boundary point
            PolygonZone rz = getRobotZone.get();
            rz.setPosition(getRobotX.get(), getRobotY.get());
            rz.setRotation(getRobotH.get());

            PolygonZone nearest = nearestZone(getRobotX.get(), getRobotY.get());
            double[] cp = closestPointOnPolygon(getRobotX.get(), getRobotY.get(), nearest);
            this.targetX = cp[0];
            this.targetY = cp[1];
        }
        this.active = a;
    }

    @Override
    public void execute() {
        double robotX = getRobotX.get();
        double robotY = getRobotY.get();
        double robotH = getRobotH.get();

        if (active) {
            // Sync robot footprint
            PolygonZone rz = getRobotZone.get();
            rz.setPosition(robotX, robotY);
            rz.setRotation(robotH);

            // Raw driver input
            double driverFwd    = getDriverFwd.get();
            double driverStrafe = getDriverStrafe.get();
            double rawMag = Math.sqrt(driverFwd * driverFwd + driverStrafe * driverStrafe);

            // Previous frame's blended output
            double prevFwd    = getBlendedFwd.get();
            double prevStrafe = getBlendedStrafe.get();
            double prevMag = Math.sqrt(prevFwd * prevFwd + prevStrafe * prevStrafe);

            // Update target if driver is intentionally adding input
            if (rawMag > 0.1 && rawMag > prevMag * 1.05) {
                PolygonZone nearest = nearestZone(robotX, robotY);
                double[] cp = closestPointOnPolygon(robotX, robotY, nearest);
                if (Double.isFinite(cp[0]) && Double.isFinite(cp[1])) {
                    this.targetX = cp[0];
                    this.targetY = cp[1];
                    if (onTargetPointChanged != null) {
                        onTargetPointChanged.accept(0.0);
                    }
                }
            }

            // Pull unit vector toward target
            double dx = targetX - robotX;
            double dy = targetY - robotY;
            double dist = Math.sqrt(dx * dx + dy * dy);

            double toZoneX, toZoneY;
            if (dist < 0.001) {
                toZoneX = 0.0;
                toZoneY = 0.0;
            } else {
                toZoneX = dx / dist;
                toZoneY = dy / dist;
            }

            // Rotate driver input to world frame
            double driverWorldX =  driverFwd * Math.cos(robotH) - driverStrafe * Math.sin(robotH);
            double driverWorldY =  driverFwd * Math.sin(robotH) + driverStrafe * Math.cos(robotH);

            // Blend pull + driver joystick
            double w = RobotHardware.VECTOR_WEIGHT_DRIVER;
            double blendedX = toZoneX * (1.0 - w) + driverWorldX * w;
            double blendedY = toZoneY * (1.0 - w) + driverWorldY * w;

            // Normalize to unit magnitude
            double mag = Math.sqrt(blendedX * blendedX + blendedY * blendedY);
            if (mag > 1.0) {
                blendedX /= mag;
                blendedY /= mag;
            }

            setBlendedFwd.accept(blendedX);
            setBlendedStrafe.accept(blendedY);
        } else {
            setBlendedFwd.accept(getDriverFwd.get());
            setBlendedStrafe.accept(getDriverStrafe.get());
        }
    }

    private PolygonZone nearestZone(double x, double y) {
        double dClose = RobotHardware.CLOSE_LAUNCH_ZONE.distanceTo(x, y);
        double dFar   = RobotHardware.FAR_LAUNCH_ZONE.distanceTo(x, y);
        return (dClose <= dFar) ? RobotHardware.CLOSE_LAUNCH_ZONE : RobotHardware.FAR_LAUNCH_ZONE;
    }

    /**
     * Returns {closestX, closestY} on the polygon boundary nearest to (px, py).
     * Iterates every edge and finds the closest point on that edge.
     */
    private double[] closestPointOnPolygon(double px, double py, PolygonZone zone) {
        Point[] verts = zone.getVertices();
        int n = verts.length;

        double bestDist = Double.MAX_VALUE;
        double bestX = px, bestY = py;

        for (int i = 0; i < n; i++) {
            Point a = verts[i];
            Point b = verts[(i + 1) % n];
            double[] cp = closestPointOnSegment(px, py, a.x, a.y, b.x, b.y);
            double d = hypot(px - cp[0], py - cp[1]);
            if (d < bestDist) {
                bestDist = d;
                bestX = cp[0];
                bestY = cp[1];
            }
        }
        return new double[]{ bestX, bestY };
    }

    /** Point on segment AB nearest to P. */
    private double[] closestPointOnSegment(
            double px, double py,
            double ax, double ay,
            double bx, double by) {
        double bxax = bx - ax;
        double byay = by - ay;
        double segLenSq = bxax * bxax + byay * byay;
        if (segLenSq < 1e-12) return new double[]{ ax, ay };
        double t = ((px - ax) * bxax + (py - ay) * byay) / segLenSq;
        t = Math.max(0.0, Math.min(1.0, t));
        return new double[]{ ax + t * bxax, ay + t * byay };
    }

    private double hypot(double dx, double dy) {
        return Math.sqrt(dx * dx + dy * dy);
    }

    @Override
    public boolean isFinished() { return false; }

    @Override
    public void end() {
        setBlendedFwd.accept(getDriverFwd.get());
        setBlendedStrafe.accept(getDriverStrafe.get());
    }
}
```

---

## 16. BaseZoneRTPCommand — Pull-to-Base-Zone

**Package:** `org.firstinspires.ftc.teamcode.commands`
**File:** `BaseZoneRTPCommand.java`

Identical algorithm to `LaunchZoneRTPCommand` but pulls toward the **center of the alliance base zone** instead of the nearest launch zone boundary.

```java
package org.firstinspires.ftc.teamcode.commands;

import com.seattlesolvers.solverslib.command.Command;
import com.skeletonarmyftc.marrow.spatial.zone.PolygonZone;
import com.skeletonarmyftc.marrow.spatial.zone.Point;
import org.firstinspires.ftc.teamcode.RobotHardware;

public class BaseZoneRTPCommand implements Command {

    private final java.util.function.Supplier<Double> getRobotX;
    private final java.util.function.Supplier<Double> getRobotY;
    private final java.util.function.Supplier<Double> getRobotH;
    private final java.util.function.Supplier<Double> getDriverFwd;
    private final java.util.function.Supplier<Double> getDriverStrafe;
    private final java.util.function.DoubleConsumer setBlendedFwd;
    private final java.util.function.DoubleConsumer setBlendedStrafe;

    private boolean active = false;

    public BaseZoneRTPCommand(
            java.util.function.Supplier<Double> getRobotX,
            java.util.function.Supplier<Double> getRobotY,
            java.util.function.Supplier<Double> getRobotH,
            java.util.function.Supplier<Double> getDriverFwd,
            java.util.function.Supplier<Double> getDriverStrafe,
            java.util.function.DoubleConsumer setBlendedFwd,
            java.util.function.DoubleConsumer setBlendedStrafe) {
        this.getRobotX = getRobotX;
        this.getRobotY = getRobotY;
        this.getRobotH = getRobotH;
        this.getDriverFwd = getDriverFwd;
        this.getDriverStrafe = getDriverStrafe;
        this.setBlendedFwd = setBlendedFwd;
        this.setBlendedStrafe = setBlendedStrafe;
    }

    public void setActive(boolean a) { this.active = a; }

    @Override
    public void execute() {
        double robotX = getRobotX.get();
        double robotY = getRobotY.get();
        double robotH = getRobotH.get();

        if (active) {
            // Select alliance base zone
            PolygonZone baseZone = RobotHardware.ALLIANCE == RobotHardware.Alliance.RED
                    ? RobotHardware.RED_BASE_ZONE
                    : RobotHardware.BLUE_BASE_ZONE;

            // Compute zone center from vertices
            Point[] verts = baseZone.getVertices();
            double cx = 0, cy = 0;
            for (Point v : verts) { cx += v.x; cy += v.y; }
            cx /= verts.length;
            cy /= verts.length;

            // Pull unit vector
            double dx = cx - robotX;
            double dy = cy - robotY;
            double mag = Math.sqrt(dx * dx + dy * dy);

            double toZoneX, toZoneY;
            if (mag < 0.001) {
                toZoneX = 0.0; toZoneY = 0.0;
            } else {
                toZoneX = dx / mag;
                toZoneY = dy / mag;
            }

            // Driver input in world frame
            double driverFwd    = getDriverFwd.get();
            double driverStrafe = getDriverStrafe.get();
            double driverWorldX =  driverFwd * Math.cos(robotH) - driverStrafe * Math.sin(robotH);
            double driverWorldY =  driverFwd * Math.sin(robotH) + driverStrafe * Math.cos(robotH);

            // Blend
            double w = RobotHardware.VECTOR_WEIGHT_DRIVER;
            double blendedX = toZoneX * (1.0 - w) + driverWorldX * w;
            double blendedY = toZoneY * (1.0 - w) + driverWorldY * w;

            // Normalize
            double totalMag = Math.sqrt(blendedX * blendedX + blendedY * blendedY);
            if (totalMag > 1.0) {
                blendedX /= totalMag;
                blendedY /= totalMag;
            }

            setBlendedFwd.accept(blendedX);
            setBlendedStrafe.accept(blendedY);
        } else {
            setBlendedFwd.accept(getDriverFwd.get());
            setBlendedStrafe.accept(getDriverStrafe.get());
        }
    }

    @Override
    public boolean isFinished() { return false; }

    @Override
    public void end() {
        setBlendedFwd.accept(getDriverFwd.get());
        setBlendedStrafe.accept(getDriverStrafe.get());
    }
}
```

---

## 17. RunToPointCommand — Pedro RTP Wrapper

**Package:** `org.firstinspires.ftc.teamcode.commands`
**File:** `RunToPointCommand.java`

Thin wrapper around `Follower.follow(Pose, holdEnd)`.

```java
package org.firstinspires.ftc.teamcode.commands;

import com.pedropathing.follower.Follower;
import com.seattlesolvers.solverslib.command.Command;
import org.firstinspires.ftc.teamcode.RobotHardware;

public class RunToPointCommand implements Command {
    private final Follower follower;
    private final double x, y, heading;
    private final boolean holdEnd;
    private final double maxSpeed;
    private boolean started = false;

    public RunToPointCommand(Follower follower, double x, double y, double heading,
                             boolean holdEnd, double maxSpeed) {
        this.follower = follower;
        this.x = x;
        this.y = y;
        this.heading = heading;
        this.holdEnd = holdEnd;
        this.maxSpeed = maxSpeed;
    }

    public RunToPointCommand(Follower follower, double x, double y, double heading) {
        this(follower, x, y, heading, true, 1.0);
    }

    @Override
    public void execute() {
        if (!started) {
            started = true;
            follower.follow(
                new com.pedropathing.geometry.Pose(x, y, heading),
                holdEnd
            );
            follower.setMaxPower(maxSpeed);
        }
    }

    @Override
    public boolean isFinished() {
        return !follower.isBusy();
    }

    @Override
    public void end() {
        if (holdEnd) {
            follower.cancelFollow();
        }
    }
}
```

---

## 18. MainTeleOp — Full TeleOp Loop

**Package:** `org.firstinspires.ftc.teamcode`
**File:** `MainTeleOp.java`
**Base class:** `SolversLib CommandOpMode`

### Gamepad Mapping Summary

**Gamepad 1 (Driver):**
| Input | Action |
|-------|--------|
| Left stick X/Y | Mecanum drive (forward/strafe) |
| Right stick X | Rotation |
| Right trigger held | `ALIGNING` → `ALIGNED` (after 0.15s on target) |
| Left trigger (edge, `ALIGNED` + ready) | Fire shot |
| Right bumper | `INTAKE` |
| Left bumper held | `INTAKE_REVERSE` (king — overrides everything) |
| Triangle / Circle | Alliance selection in `init_loop` |
| Options | Re-zero field-centric yaw to current heading (no IMU reset) |
| Touchpad | Reset pose to (72, 144) + reinit Kalman + 200ms rumble |
| Share (edge) | Re-seed pose from MegaTag |

**Gamepad 2 (Operator):**
| Input | Action |
|-------|--------|
| Dpad Up/Down | Adjust flywheel velocity offset ±50 ticks/s |
| Dpad Left/Right | Adjust hood angle offset ±0.02 |
| Circle | Instant `INTAKE` |
| Share held | Override `isReadyToShoot = true` (2 blip rumble on rising edge) |
| Left Stick Button + Right Stick Button held | Base Zone RTP |

### isReadyToShoot Conditions (all must be true)

1. State is `ALIGNED`
2. Limelight has a valid target
3. Both flywheel velocities within `FLYWHEEL_READY_TOLERANCE` of target
4. Pose distance is valid
5. Robot footprint **partially or fully inside** either launch zone

### Rumble Sequence (ready → not ready, rising edge)

```
driver.rumble(50ms)   →  operator.rumble(50ms)
pause 50ms
driver.rumble(50ms)   →  operator.rumble(50ms)
pause 1.0s (safety timeout)
```

### Full Code (with annotations)

```java
package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.Subsystem;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.seattlesolvers.solverslib.util.TelemetryData;
import com.skeletonarmyftc.marrow.util.Settings;

import org.firstinspires.ftc.teamcode.commands.BaseZoneRTPCommand;
import org.firstinspires.ftc.teamcode.commands.LaunchZoneRTPCommand;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.FlywheelSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.GateSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.HoodSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

import java.util.List;

@TeleOp(group = "main")
public class MainTeleOp extends CommandOpMode {
    private RobotHardware hw;
    private Follower follower;

    // Subsystems
    private DriveSubsystem drive;
    private FlywheelSubsystem flywheel;
    private IntakeSubsystem intake;
    private GateSubsystem gate;
    private HoodSubsystem hood;

    // State machine
    private MasterController controller;

    // Limelight
    private com.qualcomm.hardware.limelightvision.Limelight3A limelight;

    // Gamepad helpers
    private GamepadEx driver;
    private GamepadEx operator;

    // Telemetry
    private TelemetryData telemetryData;

    // Loop timing
    private ElapsedTime loopTimer = new ElapsedTime();

    // Pose persistence
    private boolean poseRestored = false;
    private final ElapsedTime poseSaveTimer = new ElapsedTime();
    private static final double POSE_SAVE_INTERVAL = 5.0;
    private static final String SETTING_POSE_X = "pose_x";
    private static final String SETTING_POSE_Y = "pose_y";
    private static final String SETTING_POSE_H = "pose_h";

    // Drive re-zero (software yaw offset, no IMU reset)
    private double driveYawOffset = 0.0;

    // Kalman reset flag — anchors filters immediately on next vision reading
    private boolean localizerJustReset = false;

    // Alliance selection
    private boolean allianceLocked = false;

    // RTP commands
    private LaunchZoneRTPCommand launchZoneRTP;
    private BaseZoneRTPCommand baseZoneRTP;

    // Shoot hood kH lock
    private boolean shootHoodLocked = false;
    private double lockedHoodPosition = 0.0;

    // Rumble state machine: 50ms on → 50ms off → 50ms on
    private enum RumblePhase { OFF, RUMBLE_1, PAUSE, RUMBLE_2 }
    private RumblePhase rumblePhase = RumblePhase.OFF;
    private final ElapsedTime rumbleTimer = new ElapsedTime();

    // Blended drive inputs
    private double blendedFwd = 0.0;
    private double blendedStrafe = 0.0;
    private boolean launchZoneActive = false;

    // ── INIT ────────────────────────────────────────────────────────
    @Override
    public void initialize() {
        super.reset();

        hw = new RobotHardware();
        hw.init(hardwareMap);

        // Rebuild Pedro constants so every restart picks up latest Panels-tuned values
        Constants.rebuild();
        follower = Constants.createFollower(hardwareMap);
        // Do NOT call follower.startTeleopDrive() — Pedro provides pose only.

        hw.initLocalizer();

        // Subsystems
        drive    = new DriveSubsystem(hw, null);
        flywheel = new FlywheelSubsystem(hw);
        intake   = new IntakeSubsystem(hw);
        gate     = new GateSubsystem(hw);
        hood     = new HoodSubsystem(hw);

        // Master controller
        controller = new MasterController(flywheel, gate, intake);
        controller.setLimelightProvider(new MasterController.LimelightProvider() {
            @Override public com.qualcomm.hardware.limelightvision.LLResult get() {
                return limelight.getLatestResult();
            }
            @Override public double getAngleToGoal() {
                var r = limelight.getLatestResult();
                return r != null ? r.getTx() : 0;
            }
            @Override public boolean hasTarget() {
                var r = limelight.getLatestResult();
                return r != null && r.isValid();
            }
        });
        controller.setPoseProvider(new MasterController.PoseProvider() {
            @Override public double getX() {
                return hw.getCorrectedX(follower.getPose().getX());
            }
            @Override public double getY() {
                return hw.getCorrectedY(follower.getPose().getY());
            }
            @Override public double getH() {
                return hw.getCorrectedH(follower.getPose().getHeading());
            }
            @Override public com.pedropathing.geometry.Point getGoalCoords() {
                return RobotHardware.GOAL_COORDS;
            }
        });
        controller.setReadyCheck(this::isReadyToShoot);
        controller.setShootDoneCallback(() -> {
            driver.resetGamepadRumble();
            operator.resetGamepadRumble();
            shootHoodLocked = false;
        });

        // Launch Zone RTP
        launchZoneRTP = new LaunchZoneRTPCommand(
            this::getRobotX,
            this::getRobotY,
            this::getRobotH,
            () -> -driver.getLeftY(),
            () -> -driver.getLeftX(),
            fwd -> blendedFwd = fwd,
            strafe -> blendedStrafe = strafe,
            () -> blendedFwd,
            () -> blendedStrafe,
            null,
            () -> RobotHardware.ROBOT_ZONE
        );

        // Base Zone RTP
        baseZoneRTP = new BaseZoneRTPCommand(
            this::getRobotX,
            this::getRobotY,
            this::getRobotH,
            () -> -driver.getLeftY(),
            () -> -driver.getLeftX(),
            fwd -> blendedFwd = fwd,
            strafe -> blendedStrafe = strafe
        );

        // Gamepad helpers
        driver   = new GamepadEx(gamepad1);
        operator = new GamepadEx(gamepad2);

        // Panels telemetry
        telemetryData = new TelemetryData(hw.panels.getTelemetry());

        // Limelight
        limelight = hardwareMap.get(
            com.qualcomm.hardware.limelightvision.Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.start();

        // Register subsystems
        registerSubsystems(List.of(drive, flywheel, intake, gate, hood));

        hw.clearBulkCache();
    }

    @Override
    public void init_loop() {
        hw.clearBulkCache();

        if (!allianceLocked) {
            GamepadEx driverInit = new GamepadEx(gamepad1);
            driverInit.readButtons();

            boolean triangleNow = driverInit.wasJustPressed(GamepadKeys.Button.TRIANGLE);
            boolean circleNow   = driverInit.wasJustPressed(GamepadKeys.Button.CIRCLE);

            if (triangleNow) {
                RobotHardware.ALLIANCE = RobotHardware.Alliance.RED;
                RobotHardware.GOAL_COORDS = RobotHardware.RED_GOAL_COORDS;
                limelight.pipelineSwitch(RobotHardware.RED_PIPELINE_INDEX);
                allianceLocked = true;
            } else if (circleNow) {
                RobotHardware.ALLIANCE = RobotHardware.Alliance.BLUE;
                RobotHardware.GOAL_COORDS = RobotHardware.BLUE_GOAL_COORDS;
                limelight.pipelineSwitch(RobotHardware.BLUE_PIPELINE_INDEX);
                allianceLocked = true;
            }
        }

        telemetry.update();
    }

    @Override
    public void start() {
        allianceLocked = true;
        controller.reset();
        controller.setState(RobotState.INTAKE);
        poseRestored = false;
        loopTimer.reset();
    }

    @Override
    public void loop() {
        hw.clearBulkCache();
        double dt = loopTimer.seconds();
        loopTimer.reset();

        // ── 1. POSE RESTORE ─────────────────────────────────────────
        if (!poseRestored) {
            restorePoseFromSettings();
        }

        // ── 2. LOCALIZATION ────────────────────────────────────────
        // updateRobotOrientation must come BEFORE follower.update()
        limelight.updateRobotOrientation(hw.getYawRadians());
        follower.update();

        double odoX = follower.getPose().getX();
        double odoY = follower.getPose().getY();
        double odoH = follower.getPose().getHeading();

        hw.predictLocalizer(dt);

        // MegaTag vision update
        var result = limelight.getLatestResult();
        boolean useResult = result != null && result.isValid();
        if (useResult) {
            if (!localizerJustReset && result.getStaleness() >= 0.1) {
                useResult = false;
            }
        }
        if (useResult) {
            double[] botpose = result.getBotpose_MT2();
            if (botpose != null && botpose.length >= 6) {
                hw.updateLocalizerFromVision(
                    odoX, odoY, odoH,
                    botpose[0], botpose[1], Math.toRadians(botpose[5])
                );
            }
            localizerJustReset = false;
        }

        double robotX = hw.getCorrectedX(odoX);
        double robotY = hw.getCorrectedY(odoY);
        double robotH = hw.getCorrectedH(odoH);

        // ── 3. GAMEPAD INPUT ───────────────────────────────────────
        driver.readButtons();
        operator.readButtons();

        // Dpad offsets (edge-triggered)
        if (operator.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
            flywheel.addOffset(RobotHardware.FLYWHEEL_VELOCITY_OFFSET_JUMP);
        }
        if (operator.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
            flywheel.addOffset(-RobotHardware.FLYWHEEL_VELOCITY_OFFSET_JUMP);
        }
        if (operator.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) {
            RobotHardware.hoodAngleOffset += RobotHardware.HOOD_ANGLE_OFFSET_JUMP;
        }
        if (operator.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) {
            RobotHardware.hoodAngleOffset -= RobotHardware.HOOD_ANGLE_OFFSET_JUMP;
        }

        // Option → re-zero field-centric yaw (software, no IMU reset)
        if (driver.wasJustPressed(GamepadKeys.Button.OPTIONS)) {
            driveYawOffset = hw.getYawRadians();
        }

        // Touchpad → full pose reset + Kalman reinit + 200ms rumble
        if (driver.wasJustPressed(GamepadKeys.Button.TOUCHPAD)) {
            double targetYaw = (RobotHardware.ALLIANCE == RobotHardware.Alliance.RED) ? 0.0 : Math.PI;
            follower.setPose(new Pose(72, 144, targetYaw));
            hw.imu.resetYaw();
            hw.initLocalizer();
            localizerJustReset = true;
            driveYawOffset = 0.0;
            driver.rumble(200);
        }

        // Share → MegaTag re-seed
        if (driver.wasJustPressed(GamepadKeys.Button.SHARE)) {
            reinitializePoseFromLimelight();
        }

        // ── 4. STATE MACHINE ───────────────────────────────────────
        controller.update(gamepad1, gamepad2, dt);
        RobotState state = controller.getState();

        // ── 5. ALIGNMENT + FALLBACK HEADING ────────────────────────
        double poseDist = getPoseDistance();

        double rotationCorrection = 0.0;
        if (result != null && result.isValid()) {
            rotationCorrection = result.getTx() * 0.05;
        } else if (state == RobotState.ALIGNING || state == RobotState.ALIGNED) {
            double odomHeadingToGoal = Math.toDegrees(
                Math.atan2(
                    RobotHardware.GOAL_COORDS.y - robotY,
                    RobotHardware.GOAL_COORDS.x - robotX
                )
            );
            double headingError = odomHeadingToGoal - Math.toDegrees(robotH);
            while (headingError > 180)  headingError -= 360;
            while (headingError < -180) headingError += 360;
            rotationCorrection = headingError * 0.05;
        }

        // ── 6. ZONE RTP ────────────────────────────────────────────
        boolean baseZoneActive = operator.isDown(GamepadKeys.Button.LEFT_STICK_BUTTON)
                             && operator.isDown(GamepadKeys.Button.RIGHT_STICK_BUTTON);

        RobotHardware.ROBOT_ZONE.setPosition(robotX, robotY);
        RobotHardware.ROBOT_ZONE.setRotation(robotH);
        boolean outsideZones = !RobotHardware.ROBOT_ZONE.isInside(RobotHardware.CLOSE_LAUNCH_ZONE)
                            && !RobotHardware.ROBOT_ZONE.isInside(RobotHardware.FAR_LAUNCH_ZONE);
        boolean rightTrigger = driver.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.1;
        launchZoneActive = rightTrigger
                && (state == RobotState.ALIGNING || state == RobotState.ALIGNED)
                && outsideZones;

        // Priority: base zone > launch zone > raw driver
        if (baseZoneActive) {
            baseZoneRTP.setActive(true);
            launchZoneRTP.setActive(false);
            baseZoneRTP.execute();
        } else if (launchZoneActive) {
            baseZoneRTP.setActive(false);
            launchZoneRTP.setActive(true);
            launchZoneRTP.execute();
        } else {
            baseZoneRTP.setActive(false);
            launchZoneRTP.setActive(false);
            blendedFwd = -driver.getLeftY();
            blendedStrafe = -driver.getLeftX();
        }

        // ── 7. FLYWHEEL ───────────────────────────────────────────
        flywheel.update(dt);

        // ── 8. HOOD ───────────────────────────────────────────────
        if (poseDist > 0) {
            double avgVel = (flywheel.getVelocityL() + flywheel.getVelocityR()) / 2.0;

            if (state == RobotState.SHOOT) {
                if (!shootHoodLocked) {
                    lockedHoodPosition = RobotHardware.hoodPosition(poseDist, RobotHardware.hoodAngleOffset);
                    shootHoodLocked = true;
                }
                double velDrop = flywheel.getTargetVelocity() - avgVel;
                double kHOffset = velDrop * RobotHardware.HOOD_COMPENSATION_COEFFICIENT;
                double finalPos = Math.max(
                    RobotHardware.HOOD_MIN_POSITION,
                    Math.min(RobotHardware.HOOD_MAX_POSITION,
                        lockedHoodPosition + kHOffset));
                hood.setPosition(finalPos);
            } else {
                shootHoodLocked = false;
                hood.setForDistance(poseDist, RobotHardware.hoodAngleOffset,
                    avgVel, flywheel.getTargetVelocity());
            }
        }

        // ── 9. DRIVE ──────────────────────────────────────────────
        double yaw = hw.getYawRadians() - driveYawOffset;

        boolean isAligning = state == RobotState.ALIGNING || state == RobotState.ALIGNED;
        double driverRot = -driver.getRightX();
        double autoRot   = rotationCorrection;
        double rot = isAligning ? driverRot + autoRot : driverRot;

        drive.driveFieldCentric(blendedFwd, blendedStrafe, rot, yaw);

        // ── 10. RUMBLE ─────────────────────────────────────────────
        boolean ready = isReadyToShoot(robotX, robotY);
        boolean shareHeld = operator.isDown(GamepadKeys.Button.SHARE);

        if (ready && !wasReady() && !shareHeld) {
            rumblePhase = RumblePhase.RUMBLE_1;
            rumbleTimer.reset();
            driver.rumble(50);
            operator.rumble(50);
        } else if (rumblePhase == RumblePhase.PAUSE && rumbleTimer.seconds() >= 0.05) {
            rumblePhase = RumblePhase.RUMBLE_2;
            rumbleTimer.reset();
            driver.rumble(50);
            operator.rumble(50);
        } else if (rumblePhase == RumblePhase.RUMBLE_2 && rumbleTimer.seconds() >= 0.05) {
            rumblePhase = RumblePhase.OFF;
            driver.stopRumble();
            operator.stopRumble();
        } else if (rumblePhase != RumblePhase.OFF && rumbleTimer.seconds() >= 1.0) {
            rumblePhase = RumblePhase.OFF;
            driver.stopRumble();
            operator.stopRumble();
        } else if (!ready && !shareHeld) {
            rumblePhase = RumblePhase.OFF;
            driver.stopRumble();
            operator.stopRumble();
        }

        // share override: 2 blips on rising edge
        if (shareHeld && !wasShareHeld()) {
            driver.rumbleBlips(2);
            operator.rumbleBlips(2);
        }
        setReadyPrev(ready, shareHeld);

        // ── 11. TELEMETRY ─────────────────────────────────────────
        sendTelemetry(result, robotX, robotY, robotH, ready, shareHeld, state, poseDist,
                baseZoneActive, launchZoneActive);

        // ── 12. POSE PERSISTENCE ───────────────────────────────────
        if (poseSaveTimer.seconds() >= POSE_SAVE_INTERVAL) {
            poseSaveTimer.reset();
            Settings.set(SETTING_POSE_X, robotX);
            Settings.set(SETTING_POSE_Y, robotY);
            Settings.set(SETTING_POSE_H, robotH);
        }
    }

    // ── HELPERS ────────────────────────────────────────────────────

    private double getPoseDistance() {
        Point goal = RobotHardware.GOAL_COORDS;
        double dx = goal.x - robotX;
        double dy = goal.y - robotY;
        double dist = Math.sqrt(dx * dx + dy * dy);
        if (dist < RobotHardware.LIMELIGHT_DIST_MIN || dist > RobotHardware.LIMELIGHT_DIST_MAX) return -1;
        return dist;
    }

    private boolean isReadyToShoot(double robotX, double robotY) {
        if (controller.getState() != RobotState.ALIGNED) return false;
        var r = limelight.getLatestResult();
        if (r == null || !r.isValid()) return false;

        RobotHardware.ROBOT_ZONE.setPosition(robotX, robotY);
        RobotHardware.ROBOT_ZONE.setRotation(robotH);
        boolean inCloseZone = RobotHardware.ROBOT_ZONE.isInside(RobotHardware.CLOSE_LAUNCH_ZONE);
        boolean inFarZone   = RobotHardware.ROBOT_ZONE.isInside(RobotHardware.FAR_LAUNCH_ZONE);
        if (!inCloseZone && !inFarZone) return false;

        double velL = flywheel.getVelocityL();
        double velR = flywheel.getVelocityR();
        double target = flywheel.getTargetVelocity();
        double tol = RobotHardware.FLYWHEEL_READY_TOLERANCE;

        boolean velocityOK = Math.abs(velL - target) < tol && Math.abs(velR - target) < tol;
        double dist = getPoseDistance();
        boolean distOK = dist > 0 && !Double.isNaN(dist);

        return velocityOK && distOK;
    }

    // Edge detection state
    private boolean prevReady = false;
    private boolean prevShare = false;
    private boolean wasReady()       { return prevReady; }
    private boolean wasShareHeld()  { return prevShare; }
    private void setReadyPrev(boolean ready, boolean share) {
        prevReady = ready;
        prevShare = share;
    }

    // Pose getters (for RTP commands)
    private double getRobotX() { return hw.getCorrectedX(follower.getPose().getX()); }
    private double getRobotY() { return hw.getCorrectedY(follower.getPose().getY()); }
    private double getRobotH() { return hw.getCorrectedH(follower.getPose().getHeading()); }

    // Pose restore: try Settings, then MegaTag
    private void restorePoseFromSettings() {
        if (poseRestored) return;
        poseRestored = true;

        double savedX = Settings.get(SETTING_POSE_X, Double.NaN);
        double savedY = Settings.get(SETTING_POSE_Y, Double.NaN);
        double savedH = Settings.get(SETTING_POSE_H, Double.NaN);

        if (!Double.isNaN(savedX) && !Double.isNaN(savedY) && !Double.isNaN(savedH)) {
            double targetYaw = (RobotHardware.ALLIANCE == RobotHardware.Alliance.RED) ? 0.0 : Math.PI;
            follower.setPose(new Pose(savedX, savedY, targetYaw));
            hw.imu.resetYaw();
            hw.initLocalizer();
            localizerJustReset = true;
            driveYawOffset = 0.0;
            return;
        }

        reinitializePoseFromLimelight();
    }

    // MegaTag re-seed
    private void reinitializePoseFromLimelight() {
        var result = limelight.getLatestResult();
        if (result == null || !result.isValid()) return;

        double[] botpose = result.getBotpose_MT2();
        if (botpose == null || botpose.length < 6) return;

        double visionX = botpose[0];
        double visionY = botpose[1];
        double visionH = Math.toRadians(botpose[5]);

        double targetYaw = (RobotHardware.ALLIANCE == RobotHardware.Alliance.RED) ? 0.0 : Math.PI;
        double yawOffset = targetYaw - visionH;
        double cosOff = Math.cos(yawOffset);
        double sinOff = Math.sin(yawOffset);
        double rotX =  visionX * cosOff + visionY * sinOff;
        double rotY = -visionX * sinOff + visionY * cosOff;

        follower.setPose(new Pose(rotX, rotY, targetYaw));
        hw.imu.resetYaw();
        hw.initLocalizer();
        localizerJustReset = true;
        driveYawOffset = 0.0;
    }

    // Telemetry
    private void sendTelemetry(com.qualcomm.hardware.limelightvision.LLResult result,
                               double robotX, double robotY, double robotH,
                               boolean ready, boolean shareOverride,
                               RobotState state, double poseDist,
                               boolean baseZoneActive, boolean launchZoneActive) {
        telemetryData.addData("Alliance", RobotHardware.ALLIANCE);
        telemetryData.addData("State", state);
        telemetryData.addData("FlywheelL vel", flywheel.getVelocityL());
        telemetryData.addData("FlywheelR vel", flywheel.getVelocityR());
        telemetryData.addData("Flywheel target", flywheel.getTargetVelocity());
        telemetryData.addData("Gate pos", gate.getPosition());
        telemetryData.addData("Hood pos", hood.getPosition());
        telemetryData.addData("isReadyToShoot", ready);
        telemetryData.addData("Share override", shareOverride);
        telemetryData.addData("Pose dist (in)", poseDist);
        telemetryData.addData("Pose X", robotX);
        telemetryData.addData("Pose Y", robotY);
        telemetryData.addData("Robot H (deg)", Math.toDegrees(robotH));
        telemetryData.addData("Vel offset", RobotHardware.flywheelVelocityOffset);
        telemetryData.addData("Hood offset", RobotHardware.hoodAngleOffset);
        telemetryData.addData("Base Zone RTP", baseZoneActive);
        telemetryData.addData("Launch Zone RTP", launchZoneActive);

        if (result != null && result.isValid()) {
            telemetryData.addData("Limelight tx", result.getTx());
            telemetryData.addData("Limelight ty", result.getTy());
        }

        telemetryData.update();
        telemetry.update();
    }

    @Override
    public void stop() {
        if (limelight != null) limelight.stop();
        drive.stop();
        flywheel.stop();
        intake.stop();
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}
```

---

## 19. Pedro Pathing Constants & Tuning

**Package:** `org.firstinspires.ftc.teamcode.pedroPathing`
**File:** `Constants.java`
**Annotation:** `@org.bylazar.ftcontrol.panels.configurables.annotations.Configurable`

### Tuning Order

| Step | Tuner | Constant |
|------|-------|----------|
| 1 | Forward Tuner | `FORWARD_TICKS_TO_INCHES` |
| 2 | Lateral Tuner | `STRAFE_TICKS_TO_INCHES` |
| 3 | Offsets Tuner | `FORWARD_POD_Y`, `STRAFE_POD_X` |
| 4 | Forward Velocity Tuner | `X_VELOCITY` |
| 5 | Lateral Velocity Tuner | `Y_VELOCITY` |
| 6 | Forward ZPA Tuner | `FORWARD_ZPA` |
| 7 | Lateral ZPA Tuner | `LATERAL_ZPA` |
| 8 | Heading Tuner | `HEADING_P/I/D/F` |
| 9 | Translational Tuner | `TRANSLATIONAL_P/I/D/F` |
| 10 | Drive Tuner | `DRIVE_P/I/D/F/T`, `BRAKING_STRENGTH` |
| 11 | Centripetal Tuner | `CENTRIPETAL_SCALING` |

### Full Code

```java
package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.localization.TwoWheelLocalizerConstants;
import com.pedropathing.pathgen.PathConstraints;
import com.pedropathing.util.RawEncoder;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

@org.bylazar.ftcontrol.panels.configurables.annotations.Configurable
public class Constants {

    // ═══ DRIVE MOTOR NAMES (match RC config XML) ══════════════════
    public static String LEFT_FRONT_MOTOR  = "FL";
    public static String LEFT_REAR_MOTOR   = "BL";
    public static String RIGHT_FRONT_MOTOR = "FR";
    public static String RIGHT_REAR_MOTOR  = "BR";

    // ═══ DRIVE MOTOR DIRECTIONS ══════════════════════════════════
    public static DcMotorSimple.Direction LEFT_FRONT_DIRECTION  = DcMotorSimple.Direction.FORWARD;
    public static DcMotorSimple.Direction LEFT_REAR_DIRECTION   = DcMotorSimple.Direction.FORWARD;
    public static DcMotorSimple.Direction RIGHT_FRONT_DIRECTION = DcMotorSimple.Direction.REVERSE;
    public static DcMotorSimple.Direction RIGHT_REAR_DIRECTION  = DcMotorSimple.Direction.REVERSE;

    // ═══ ODOMETRY POD NAMES ══════════════════════════════════════
    // forwardEncoder  — pod parallel to robot forward (FL motor)
    // strafeEncoder   — pod perpendicular to forward (BR motor)
    public static String FORWARD_ENCODER_HWMAP_NAME = "FL";
    public static String STRAFE_ENCODER_HWMAP_NAME  = "BR";

    public static RawEncoder.Direction FORWARD_ENCODER_DIRECTION  = RawEncoder.Direction.FORWARD;
    public static RawEncoder.Direction STRAFE_ENCODER_DIRECTION   = RawEncoder.Direction.FORWARD;

    // ═══ TICKS-TO-INCHES MULTIPLIERS ═════════════════════════════
    // Populated by Forward / Lateral Tuners
    public static double FORWARD_TICKS_TO_INCHES = 1.0;
    public static double STRAFE_TICKS_TO_INCHES  = 1.0;

    // ═══ POD OFFSETS (inches from center of rotation) ═════════════
    // Run Offsets Tuner to find these
    public static double FORWARD_POD_Y = 0.0;
    public static double STRAFE_POD_X  = 0.0;

    // ═══ IMU ══════════════════════════════════════════════════════
    public static String IMU_HWMAP_NAME = "imu";
    public static RevHubOrientationOnRobot.LogoFacingDirection IMU_LOGO_FACING =
            RevHubOrientationOnRobot.LogoFacingDirection.UP;
    public static RevHubOrientationOnRobot.UsbFacingDirection IMU_USB_FACING =
            RevHubOrientationOnRobot.UsbFacingDirection.LEFT;

    // ═══ PATH CONSTRAINTS ═════════════════════════════════════════
    public static double PATH_MAX_VELOCITY      = 40.0;   // in/s
    public static double PATH_MAX_ACCELERATION  = 60.0;   // in/s²
    public static double PATH_MAX_JERK         = 120.0;   // in/s³
    public static double PATH_MAX_ANG_VELOCITY     = Math.toRadians(180);  // rad/s
    public static double PATH_MAX_ANG_ACCELERATION = Math.toRadians(360);  // rad/s²

    // ═══ ROBOT MASS ══════════════════════════════════════════════
    public static double ROBOT_MASS_KG = 5.0;

    // ═══ VELOCITY MULTIPLIERS ════════════════════════════════════
    // Populated by Velocity Tuners
    public static double X_VELOCITY = 1.0;
    public static double Y_VELOCITY = 1.0;

    // ═══ ZERO-POWER ACCELERATION ═════════════════════════════════
    // Populated by ZPA Tuners
    public static double FORWARD_ZPA = 50.0;
    public static double LATERAL_ZPA = 50.0;

    // ═══ CENTRIPETAL SCALING ═════════════════════════════════════
    // Populated by Centripetal Tuner
    public static double CENTRIPETAL_SCALING = 0.005;

    // ═══ HEADING PIDF ════════════════════════════════════════════
    public static double HEADING_P = 3.0;
    public static double HEADING_I = 0.0;
    public static double HEADING_D = 0.1;
    public static double HEADING_F = 0.0;

    // ═══ TRANSLATIONAL PIDF ═══════════════════════════════════════
    public static double TRANSLATIONAL_P = 1.5;
    public static double TRANSLATIONAL_I = 0.0;
    public static double TRANSLATIONAL_D = 0.05;
    public static double TRANSLATIONAL_F = 0.0;

    // ═══ DRIVE PIDF ═══════════════════════════════════════════════
    public static double DRIVE_P = 0.1;
    public static double DRIVE_I = 0.0;
    public static double DRIVE_D = 0.01;
    public static double DRIVE_F = 0.0;
    public static double DRIVE_T = 0.6;  // derivative filter time constant

    public static double BRAKING_STRENGTH = 0.5;

    // ═══ SECONDARY PIDFs (optional — enable with .useSecondary*PIDF) ══
    public static double HEADING_SECONDARY_P = 1.5;
    public static double HEADING_SECONDARY_I = 0.0;
    public static double HEADING_SECONDARY_D = 0.05;
    public static double HEADING_SECONDARY_F = 0.0;

    public static double TRANSLATIONAL_SECONDARY_P = 0.75;
    public static double TRANSLATIONAL_SECONDARY_I = 0.0;
    public static double TRANSLATIONAL_SECONDARY_D = 0.02;
    public static double TRANSLATIONAL_SECONDARY_F = 0.0;

    public static double DRIVE_SECONDARY_P = 0.05;
    public static double DRIVE_SECONDARY_I = 0.0;
    public static double DRIVE_SECONDARY_D = 0.005;
    public static double DRIVE_SECONDARY_F = 0.0;
    public static double DRIVE_SECONDARY_T = 0.6;

    // ═══ DRIVE KALMAN FILTER ══════════════════════════════════════
    public static double DRIVE_KALMAN_MODEL_COVARIANCE = 6.0;
    public static double DRIVE_KALMAN_DATA_COVARIANCE = 1.0;

    // ═══ BUILT OBJECTS ════════════════════════════════════════════
    public static TwoWheelLocalizerConstants localizerConstants;
    public static FollowerConstants followerConstants;
    public static PathConstraints pathConstraints;

    static { rebuild(); }

    /** Regenerate all built objects. Call whenever a constant changes. */
    public static void rebuild() {
        localizerConstants =
            new TwoWheelLocalizerConstants()
                .forwardEncoder_HardwareMapName(FORWARD_ENCODER_HWMAP_NAME)
                .strafeEncoder_HardwareMapName(STRAFE_ENCODER_HWMAP_NAME)
                .IMU_HardwareMapName(IMU_HWMAP_NAME)
                .IMU_Orientation(new RevHubOrientationOnRobot(IMU_LOGO_FACING, IMU_USB_FACING))
                .forwardEncoderDirection(FORWARD_ENCODER_DIRECTION)
                .strafeEncoderDirection(STRAFE_ENCODER_DIRECTION)
                .forwardTicksToInches(FORWARD_TICKS_TO_INCHES)
                .strafeTicksToInches(STRAFE_TICKS_TO_INCHES)
                .forwardPodY(FORWARD_POD_Y)
                .strafePodX(STRAFE_POD_X);

        followerConstants =
            new FollowerConstants()
                .mass(ROBOT_MASS_KG)
                .maxPower(1.0)
                .xVelocity(X_VELOCITY)
                .yVelocity(Y_VELOCITY)
                .forwardZeroPowerAcceleration(FORWARD_ZPA)
                .lateralZeroPowerAcceleration(LATERAL_ZPA)
                .centripetalScaling(CENTRIPETAL_SCALING)
                .headingPIDFCoefficients(new com.pedropathing.util.PIDFCoefficients(
                        HEADING_P, HEADING_I, HEADING_D, HEADING_F))
                .translationalPIDFCoefficients(new com.pedropathing.util.PIDFCoefficients(
                        TRANSLATIONAL_P, TRANSLATIONAL_I, TRANSLATIONAL_D, TRANSLATIONAL_F))
                .drivePIDFCoefficients(new com.pedropathing.util.FilteredPIDFCoefficients(
                        DRIVE_P, DRIVE_I, DRIVE_D, DRIVE_T, DRIVE_F))
                .brakingStrength(BRAKING_STRENGTH)
                .driveKalmanFilterParameters(DRIVE_KALMAN_MODEL_COVARIANCE, DRIVE_KALMAN_DATA_COVARIANCE);

        pathConstraints =
            new PathConstraints(
                PATH_MAX_VELOCITY,
                PATH_MAX_ACCELERATION,
                PATH_MAX_JERK,
                PATH_MAX_ANG_VELOCITY,
                PATH_MAX_ANG_ACCELERATION
            );
    }

    /**
     * Factory — creates a fully-configured Pedro Follower.
     * Call once per OpMode in initialize().
     */
    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .twoWheelLocalizer(localizerConstants)
                .mecanumDrivetrain(
                    new com.pedropathing.drive.MecanumConstants()
                        .maxPower(1.0)
                        .leftFrontMotorName(LEFT_FRONT_MOTOR)
                        .leftRearMotorName(LEFT_REAR_MOTOR)
                        .rightFrontMotorName(RIGHT_FRONT_MOTOR)
                        .rightRearMotorName(RIGHT_REAR_MOTOR)
                        .leftFrontMotorDirection(LEFT_FRONT_DIRECTION)
                        .leftRearMotorDirection(LEFT_REAR_DIRECTION)
                        .rightFrontMotorDirection(RIGHT_FRONT_DIRECTION)
                        .rightRearMotorDirection(RIGHT_REAR_DIRECTION)
                )
                .build();
    }
}
```

---

## 20. Pedro Pathing Tuning OpModes

**Package:** `org.firstinspires.ftc.teamcode.pedroPathing`
**File:** `Tuning.java`

Pedro Pathing provides a suite of pre-built tuning OpModes. They are registered via the `changes()` / `drawCurrent()` / `drawCurrentAndHistory()` API. Import the static members:

```java
import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.changes;
import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.drawCurrent;
import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.drawCurrentAndHistory;
import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;
import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.stopRobot;
import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.telemetryM;
```

The tuning OpModes use `SelectableOpMode` from Pedro to create a tuning menu. Each OpMode in the file corresponds to one tuning step from §19. Refer to the Pedro Pathing documentation for the exact button sequences for each tuner.

---

## 21. Control Flow Diagrams

### TeleOp Loop Order (every `loop()` tick)

```
1. clearBulkCache()
2. dt = loopTimer.seconds(); loopTimer.reset()

3. POSE RESTORE
   └─ if (!poseRestored) restorePoseFromSettings()
      ├─ Try Marrow Settings.get(pose_x/y/h)
      │  └─ if valid → setPose() + initLocalizer() + localizerJustReset=true
      └─ else → reinitializePoseFromLimelight()

4. LOCALIZATION
   └─ limelight.updateRobotOrientation(hw.getYawRadians())
   └─ follower.update()
   └─ hw.predictLocalizer(dt)
   └─ If MegaTag valid + fresh:
      └─ hw.updateLocalizerFromVision(odoX/Y/H, botpose)
   └─ robotX/Y/H = hw.getCorrectedX/Y/H(odoX/Y/H)

5. GAMEPAD INPUT
   ├─ driver.readButtons()
   ├─ operator.readButtons()
   ├─ Dpad → adjust offsets (edge-triggered)
   ├─ Options → driveYawOffset = currentYaw
   ├─ Touchpad → full pose reset + Kalman reinit + rumble
   └─ Share → reinitializePoseFromLimelight()

6. STATE MACHINE
   └─ controller.update(gamepad1, gamepad2, dt)
      └─ Handles all transitions per priority rules

7. ALIGNMENT
   ├─ rotationCorrection = limelight.tx × 0.05   (P on crosshair)
   └─ Fallback: odometry heading toward GOAL_COORDS

8. ZONE RTP
   ├─ Base Zone active? (gamepad2 stick buttons)
   ├─ Launch Zone active? (right trigger + aligning + outside zones)
   └─ Priority: base > launch > raw driver

9. FLYWHEEL
   └─ flywheel.update(dt)

10. HOOD
    ├─ Non-SHOOT: hood.setForDistance(poseDist, offset, avgVel, targetVel)
    └─ SHOOT: lock base + kH compensation

11. DRIVE
    └─ drive.driveFieldCentric(blendedFwd, blendedStrafe, rot, yaw)

12. RUMBLE
    ├─ Rising edge of isReadyToShoot → 50ms-on/50ms-pause/50ms-on pattern
    └─ Share held rising edge → 2 blips

13. TELEMETRY
    └─ telemetryData.addData(...) + update()

14. POSE PERSISTENCE
    └─ Every 5s → Settings.set(pose_x/y/h)
```

---

## 22. Tuning Constants Quick Reference

### RobotHardware (live-tunable via Panels)

| Constant | Default | Unit | Description |
|----------|---------|------|-------------|
| `FLYWHEEL_TARGET_VELOCITY` | 2800 | ticks/s | Flywheel speed target |
| `FLYWHEEL_READY_TOLERANCE` | 150 | ticks/s | Velocity tolerance for `isReadyToShoot` |
| `FLYWHEEL_L_KP/KI/KD/KF` | 0.0001/0.001/0/0 | — | Left flywheel PIDF |
| `FLYWHEEL_R_KP/KI/KD/KF` | 0.0001/0.001/0/0 | — | Right flywheel PIDF |
| `FLYWHEEL_VELOCITY_OFFSET_JUMP` | 50 | ticks/s | Dpad up/down increment |
| `FLYWHEEL_TPR` | 384.5 | ticks/rev | Encoder ticks per revolution |
| `FLYWHEEL_K_EMF` | ~0.0186 | V/(rad/s) | Back-EMF constant |
| `FLYWHEEL_MAX_INTEGRAL_VOLTAGE` | 3.0 | V | I-term ceiling |
| `SHOOT_DELAY` | 2.5 | s | Non-blocking shoot timer |
| `GATE_OPEN_POSITION` | 0.55 | servo units | Gate open position |
| `GATE_CLOSE_POSITION` | 0.0 | servo units | Gate closed position |
| `HOOD_MIN/MAX_POSITION` | 0.05/0.80 | servo units | Hardstop positions |
| `HOOD_ANGLE_OFFSET_JUMP` | 0.02 | servo units | Dpad left/right increment |
| `HOOD_COMPENSATION_COEFFICIENT` | 0.0005 | — | kH velocity-drop factor |
| `ALIGNMENT_DELAY` | 0.15 | s | Crosshair on-target time to ALIGNED |
| `DRIVE_MAX_CURRENT` | 3.0 | A | Drive voltage clamp ceiling |
| `DRIVE_STALL_CURRENT_THRESHOLD` | 2.5 | A | Stall detection threshold |
| `DRIVE_INPUT_CURVE_EXP` | 1.5 | — | Joystick exponential curve |
| `VECTOR_WEIGHT_DRIVER` | 0.7 | — | RTP driver authority (0=full pull, 1=full driver) |
| `KALMAN_Q_BIAS_X/Y/H` | 0.02/0.02/0.005 | variance/√s | Kalman process noise |
| `KALMAN_R_VISION_X/Y/H` | 4.0/4.0/0.01 | variance | Kalman measurement noise |
| `LIMELIGHT_DIST_MIN/MAX` | 5.0/120.0 | in | Valid distance range |
| `ROBOT_SIZE_INCHES` | 18.0 | in | Robot footprint size |
| `INTAKE_POWER` | 10.0 | V | Intake motor voltage |
| `ALLIANCE` | BLUE | — | Selected alliance |

### Pedro Pathing Constants (`pedroPathing/Constants.java`)

| Constant | Default | Tuned By |
|----------|---------|----------|
| `FORWARD_TICKS_TO_INCHES` | 1.0 | Forward Tuner |
| `STRAFE_TICKS_TO_INCHES` | 1.0 | Lateral Tuner |
| `FORWARD_POD_Y` | 0.0 in | Offsets Tuner |
| `STRAFE_POD_X` | 0.0 in | Offsets Tuner |
| `X_VELOCITY` | 1.0 in/s | Forward Velocity Tuner |
| `Y_VELOCITY` | 1.0 in/s | Lateral Velocity Tuner |
| `FORWARD_ZPA` | 50.0 in/s² | Forward ZPA Tuner |
| `LATERAL_ZPA` | 50.0 in/s² | Lateral ZPA Tuner |
| `HEADING_P/I/D/F` | 3.0/0.0/0.1/0.0 | Heading Tuner |
| `TRANSLATIONAL_P/I/D/F` | 1.5/0.0/0.05/0.0 | Translational Tuner |
| `DRIVE_P/I/D/F/T` | 0.1/0.0/0.01/0.0/0.6 | Drive Tuner |
| `BRAKING_STRENGTH` | 0.5 | Drive Tuner |
| `CENTRIPETAL_SCALING` | 0.005 | Centripetal Tuner |
| `ROBOT_MASS_KG` | 5.0 kg | Measured |
| `PATH_MAX_VELOCITY` | 40.0 in/s | Field tuning |
| `PATH_MAX_ACCELERATION` | 60.0 in/s² | Field tuning |
| `DRIVE_KALMAN_MODEL_COVARIANCE` | 6.0 | Drive Tuner |
| `DRIVE_KALMAN_DATA_COVARIANCE` | 1.0 | Drive Tuner |
