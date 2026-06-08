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

/**
 * All hardware declarations and tuned constants live here.
 * Panels Configurables are declared as {@code public static} fields so live tuning
 * works without redeploying code.
 */
@org.bylazar.ftcontrol.panels.configurables.annotations.Configurable
public class RobotHardware {

    // ─────────────────────────────────────────────────────────────
    // ALLIANCE & FIELD GEOMETRY
    // ─────────────────────────────────────────────────────────────

    /** Selected by driver in INIT via gamepad buttons. */
    public static Alliance ALLIANCE = Alliance.BLUE;

    public enum Alliance { RED, BLUE }

    /** Goal position in Pedro coordinates (origin = bottom-left, [0,144]).
     *  Switched at runtime in init_loop() to select the correct alliance target. */
    public static Point GOAL_COORDS = new Point(144, 72);

    /** Red alliance goal coordinates (Pedro field, origin = bottom-left). */
    public static Point RED_GOAL_COORDS   = new Point(144, 144);

    /** Blue alliance goal coordinates (Pedro field, origin = bottom-left). */
    public static Point BLUE_GOAL_COORDS  = new Point(0, 144);

    // ─────────────────────────────────────────────────────────────
    // LAUNCH ZONE POLYGONS (Marrow PolygonZone) — pull-to-zone RTP
    // ─────────────────────────────────────────────────────────────

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

    // ─────────────────────────────────────────────────────────────
    // BASE ZONE POLYGONS (Marrow PolygonZone) — pull-to-base RTP
    // ─────────────────────────────────────────────────────────────

    /**
     * Blue alliance base zone — 20×20 in² square centered at (105.5, 33.5).
     * Represent the robot as an 18×18 in² zone for collision padding.
     */
    public static final com.skeletonarmyftc.marrow.spatial.zone.PolygonZone BLUE_BASE_ZONE =
            new com.skeletonarmyftc.marrow.spatial.zone.PolygonZone(
                    new com.skeletonarmyftc.marrow.spatial.zone.Point(105.5, 33.5),
                    20, 20
            );

    /**
     * Red alliance base zone — 20×20 in² square centered at (38.5, 33.5).
     * Represent the robot as an 18×18 in² zone for collision padding.
     */
    public static final com.skeletonarmyftc.marrow.spatial.zone.PolygonZone RED_BASE_ZONE =
            new com.skeletonarmyftc.marrow.spatial.zone.PolygonZone(
                    new com.skeletonarmyftc.marrow.spatial.zone.Point(38.5, 33.5),
                    20, 20
            );

    /**
     * Robot footprint size in inches — both width and length (square bot).
     * Adjust this to match your actual robot dimensions; used to build ROBOT_ZONE
     * and for all perimeter-aware zone checks.
     */
    public static double ROBOT_SIZE_INCHES = 18.0;

    /**
     * Robot collision zone — square polygon representing the robot's physical footprint.
     * Position and rotation must be updated every loop to track the live robot pose.
     * Recreated whenever ROBOT_SIZE_INCHES changes.
     */
    public static com.skeletonarmyftc.marrow.spatial.zone.PolygonZone ROBOT_ZONE =
            new com.skeletonarmyftc.marrow.spatial.zone.PolygonZone(ROBOT_SIZE_INCHES, ROBOT_SIZE_INCHES);

    // ─────────────────────────────────────────────────────────────
    // FLYWHEEL
    // ─────────────────────────────────────────────────────────────

    /**
     * Switch between flywheel control modes:
     *   true  = FlywheelSubsystem     — custom voltage-loop with back-EMF compensation
     *   false = FlywheelSubsystemSimple — SolversLib MotorEx built-in PIDF
     */
    public static final boolean FLYWHEEL_USE_VOLTAGE_LOOP = false;

    public static double FLYWHEEL_TARGET_VELOCITY = 2800.0; // ticks/sec per motor
    public static double FLYWHEEL_READY_TOLERANCE = 150.0;  // ticks/sec — isReadyToShoot threshold

    // PIDF for flywheelL (left motor)
    public static double FLYWHEEL_L_KP = 0.0001;
    public static double FLYWHEEL_L_KI = 0.001;
    public static double FLYWHEEL_L_KD = 0.0;
    public static double FLYWHEEL_L_KF = 0.0;

    // PIDF for flywheelR (right motor)
    public static double FLYWHEEL_R_KP = 0.0001;
    public static double FLYWHEEL_R_KI = 0.001;
    public static double FLYWHEEL_R_KD = 0.0;
    public static double FLYWHEEL_R_KF = 0.0;

    // Flywheel velocity offset increments (gamepad2 dpad)
    public static double FLYWHEEL_VELOCITY_OFFSET_JUMP = 50.0;

    /**
     * Encoder ticks per revolution for the MR encoder on GoBilda 5202 motors.
     * Used by the voltage-loop subsystem to convert ticks/sec → rad/s.
     */
    public static double FLYWHEEL_TPR = 384.5;

    public static double FLYWHEEL_K_EMF = (12.0 - 0.326) / (6000.0 * 2.0 * Math.PI / 60.0); // V/(rad/s) ≈ 0.0186

    /**
     * Maximum voltage contribution from the I-term. Prevents the integral from
     * accumulating beyond this regardless of ki. Keeps integral wind-up bounded
     * and ki-independent.
     */
    public static double FLYWHEEL_MAX_INTEGRAL_VOLTAGE = 3.0; // volts

    // ─────────────────────────────────────────────────────────────
    // SHOOT SEQUENCE
    // ─────────────────────────────────────────────────────────────

    public static double SHOOT_DELAY = 2.5; // seconds — non-blocking via ElapsedTime

    // ─────────────────────────────────────────────────────────────
    // GATE SERVO
    // ─────────────────────────────────────────────────────────────

    public static double GATE_OPEN_POSITION  = 0.55;
    public static double GATE_CLOSE_POSITION = 0.0;

    // ─────────────────────────────────────────────────────────────
    // HOOD
    // ─────────────────────────────────────────────────────────────

    /** Hardstop positions (raw servo units 0–1). */
    public static double HOOD_MIN_POSITION = 0.05;
    public static double HOOD_MAX_POSITION = 0.80;

    /** Hood angle offset increment (gamepad2 dpad). */
    public static double HOOD_ANGLE_OFFSET_JUMP = 0.02;

    /**
     * Lookup table: distance from goal (inches) → hood servo position.
     * Data sourced from experimental measurement per the hooded-shooter skill.
     * Replace with real measured values from your robot.
     */
    public static double[] HOOD_DISTANCE_SAMPLES = {
        30.0, 40.0, 50.0, 60.0, 70.0, 80.0
    };
    public static double[] HOOD_POSITION_SAMPLES = {
        0.75, 0.65, 0.55, 0.45, 0.35, 0.25
    };

    /** Compensation coefficient for hood angle when flywheel velocity drops. */
    public static double HOOD_COMPENSATION_COEFFICIENT = 0.0005;

    // ─────────────────────────────────────────────────────────────
    // LIMELIGHT
    // ─────────────────────────────────────────────────────────────

    public static int RED_PIPELINE_INDEX   = 0;
    public static int BLUE_PIPELINE_INDEX  = 1;

    /** Acceptable distance range from Limelight (inches). */
    public static double LIMELIGHT_DIST_MIN = 5.0;
    public static double LIMELIGHT_DIST_MAX = 120.0;

    /** Camera mounting angle (degrees from horizontal). */
    public static double LIMELIGHT_MOUNT_ANGLE = 25.0;
    /** Fixed offset added to Limelight distance reading. */
    public static double LIMELIGHT_DISTANCE_OFFSET = 0.0;
    /** Goal height above the field floor for distance trigonometry (inches). */
    public static double GOAL_HEIGHT = 18.0;

    // ─────────────────────────────────────────────────────────────
    // ALIGNMENT
    // ─────────────────────────────────────────────────────────────

    /**
     * Seconds the limelight crosshair must stay aligned before state transitions
     * from ALIGNING → ALIGNED.
     */
    public static double ALIGNMENT_DELAY = 0.15;

    // ─────────────────────────────────────────────────────────────
    // DRIVE MOTOR CURRENT LIMITING
    // ─────────────────────────────────────────────────────────────

    public static double DRIVE_MAX_CURRENT               = 3.0; // amps — voltage clamping ceiling
    public static double DRIVE_STALL_CURRENT_THRESHOLD    = 2.5; // amps — triggers torque limiting
    public static int    CURRENT_CHECK_INTERVAL       = 30;   // poll current every N loops when not stalling

    // ─────────────────────────────────────────────────────────────
    // DRIVETRAIN INPUT CURVE
    // ─────────────────────────────────────────────────────────────

    /** Exponent for the drive joystick input curve. 1.0 = linear. */
    public static double DRIVE_INPUT_CURVE_EXP = 1.5;

    // ─────────────────────────────────────────────────────────────
    // DRIVETRAIN VECTOR ADDITION (launch zone RTP)
    // ─────────────────────────────────────────────────────────────

    /** How strongly the driver joystick overrides the auto pull toward the launch zone. */
    public static double VECTOR_WEIGHT_DRIVER = 0.7;

    // ─────────────────────────────────────────────────────────────
    // KALMAN LOCALIZER TUNING
    // ─────────────────────────────────────────────────────────────

    /** Bias random-walk standard deviation per sqrt(second). */
    public static double KALMAN_Q_BIAS_X = 0.02;
    public static double KALMAN_Q_BIAS_Y = 0.02;
    public static double KALMAN_Q_BIAS_H = 0.005;

    /** Vision measurement variance. */
    public static double KALMAN_R_VISION_X = 4.0;
    public static double KALMAN_R_VISION_Y = 4.0;
    public static double KALMAN_R_VISION_H = 0.01;

    // ─────────────────────────────────────────────────────────────
    // ACTUAL HARDWARE — do not tune via Panels
    // ─────────────────────────────────────────────────────────────

    public MotorEx fl, fr, bl, br;
    public MotorEx flywheelL, flywheelR;
    public MotorEx odomPara;       // parallel pod — forward encoder (measures forward/straight)
    public MotorEx odomPerpend;    // perpendicular pod — lateral encoder (measures strafing)
    public Motor intake;
    public ServoExGroup hood;
    public ServoEx gate;
    public IMU imu;
    public LynxModule controlHub;
    public VoltageSensor voltageSensor;

    // Panels TelemetryManager — set once per OpMode
    public org.bylazar.ftcontrol.panels.Panels panels;
    public com.seattlesolvers.solverslib.util.TelemetryData telemetryData;

    // Live flywheel velocity offset (modified by gamepad2 dpad) — static for tuning access
    public static double flywheelVelocityOffset = 0.0;

    // Live hood angle offset (modified by gamepad2 dpad) — static for tuning access
    public static double hoodAngleOffset = 0.0;


    /**
     * Intake motor power in volts. Battery compensation scales this automatically
     * so the intake runs at consistent speed regardless of battery voltage.
     */
    public static double INTAKE_POWER = 10.0;

    // ─────────────────────────────────────────────────────────────
    // INIT
    // ─────────────────────────────────────────────────────────────

    public void init(@NonNull HardwareMap hwMap) {
        // Drive motors
        fl = new MotorEx(hwMap, "FL", GoBILDA.RPM_435);
        fr = new MotorEx(hwMap, "FR", GoBILDA.RPM_435);
        bl = new MotorEx(hwMap, "BL", GoBILDA.RPM_435);
        br = new MotorEx(hwMap, "BR", GoBILDA.RPM_435);

        fr.setInverted(true);
        br.setInverted(true);

        // Drivetrain motors: BRAKE so they resist motion when sticks center
        fl.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        // Odometry pods — dedicated encoder motors for Two-Wheel Localizer
        // These should be on the two fastest encoder ports (0 and 3 on REV Control Hub)
        odomPara    = new MotorEx(hwMap, "FL");
        odomPerpend = new MotorEx(hwMap, "BR");

        odomPara.setInverted(false);
        odomPerpend.setInverted(false);

        // Flywheel motors — MotorEx in VelocityControl.
        // Feedforward: kV = 12 / maxVelocity so set(1.0) = max speed;
        // kS = 0.15 V overcomes static friction.
        flywheelL = new MotorEx(hwMap, "flywheelL");
        flywheelR = new MotorEx(hwMap, "flywheelR");
        flywheelL.setRunMode(Motor.RunMode.VelocityControl);
        flywheelR.setRunMode(Motor.RunMode.VelocityControl);

        // 384.5 ticks/rev × 6000 RPM / 60 = 38450 ticks/s
        double flyKV = 12.0 / (384.5 * 6000.0 / 60.0);
        flywheelL.setFeedforwardCoefficients(0.15, flyKV);
        flywheelR.setFeedforwardCoefficients(0.15, flyKV);

        // Intake motor — no encoder. Driven via setPower with battery voltage compensation.
        intake = new Motor(hwMap, "intake");
        intake.setInverted(false);


        // Phase 2 — reverse the right hood servo so the group can accept a single value
        ServoEx _hoodL = new ServoEx(hwMap, "hoodL");
        ServoEx _hoodR = new ServoEx(hwMap, "hoodR");
        _hoodR.setInverted(true);
        hood = new ServoExGroup(_hoodL, _hoodR);
        gate = new ServoEx(hwMap, "gate");
        gate.setPosition(GATE_CLOSE_POSITION);

        // IMU — orientation read from pedroPathing.Constants so Pedro's TwoWheelLocalizer
        // and RobotHardware use the same values. Update IMU_LOGO_FACING / IMU_USB_FACING
        // in Constants.java to change both at once.
        imu = hwMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(
                org.firstinspires.ftc.teamcode.pedroPathing.Constants.IMU_LOGO_FACING,
                org.firstinspires.ftc.teamcode.pedroPathing.Constants.IMU_USB_FACING
        );
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        // Limelight — initialized by the OpMode that uses it

        // Control Hub for bulk reads
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

        // Voltage sensor
        voltageSensor = hwMap.voltageSensor.get("Control Hub");

        // Panels
        panels = org.bylazar.ftcontrol.panels.Panels.getInstance();
        telemetryData = new com.seattlesolvers.solverslib.util.TelemetryData(panels.getTelemetry());
    }

    // ─────────────────────────────────────────────────────────────
    // VOLTAGE COMPENSATION
    // ─────────────────────────────────────────────────────────────

    /** Returns current battery voltage. */
    public double batteryVoltage() {
        return voltageSensor.getVoltage();
    }

    /** Returns current IMU yaw in radians. */
    public double getYawRadians() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }

    // ─────────────────────────────────────────────────────────────
    // INPUT SHAPING
    // ─────────────────────────────────────────────────────────────

    /**
     * Applies an exponential input curve to a joystick value.
     * exp = 1.0 → linear; exp > 1.0 → more sensitive near zero.
     */
    public static double applyInputCurve(double input, double exp) {
        return Math.copySign(Math.pow(Math.abs(input), exp), input);
    }

    // ─────────────────────────────────────────────────────────────
    // KALMAN LOCALIZER — 1D drift filter per axis
    // ─────────────────────────────────────────────────────────────

    public static class DriftFilter {
        public double drift = 0.0;
        public double P     = 1.0;
        public final double qBias;
        public final double rVision;

        public DriftFilter(double qBias, double rVision) {
            this.qBias   = qBias;
            this.rVision = rVision;
        }

        /** Predict step — call every loop. */
        public void predict(double dt) {
            P += qBias * qBias * dt;
        }

        /** Update step — call only when fresh vision reading is available. */
        public void update(double odomReading, double visionReading) {
            double measuredDrift = odomReading - visionReading;
            double K = P / (P + rVision);
            drift = drift + K * (measuredDrift - drift);
            P = (1.0 - K) * P;
        }

        public double corrected(double odomReading) {
            return odomReading - drift;
        }
    }

    public DriftFilter filterX, filterY, filterH;

    public void initLocalizer() {
        filterX = new DriftFilter(KALMAN_Q_BIAS_X, KALMAN_R_VISION_X);
        filterY = new DriftFilter(KALMAN_Q_BIAS_Y, KALMAN_R_VISION_Y);
        filterH = new DriftFilter(KALMAN_Q_BIAS_H, KALMAN_R_VISION_H);
    }

    /**
     * Predict step — call every loop.
     * @param dt seconds since last call
     */
    public void predictLocalizer(double dt) {
        filterX.predict(dt);
        filterY.predict(dt);
        filterH.predict(dt);
    }

    /**
     * Update step — call when a fresh, valid MegaTag 2 reading is available.
     */
    public void updateLocalizerFromVision(double odomX, double odomY, double odomH,
                                          double visionX, double visionY, double visionH) {
        filterX.update(odomX, visionX);
        filterY.update(odomY, visionY);
        filterH.update(odomH, visionH);
    }

    public double getCorrectedX(double odomX) { return filterX.corrected(odomX); }
    public double getCorrectedY(double odomY) { return filterY.corrected(odomY); }
    public double getCorrectedH(double odomH) { return filterH.corrected(odomH); }

    // ─────────────────────────────────────────────────────────────
    // HOOD LUT
    // ─────────────────────────────────────────────────────────────

    /** Linear-interpolated lookup: distance → hood servo position. */
    public static class HoodLUT {
        private final double[] dist;
        private final double[] pos;

        public HoodLUT(double[] distSamples, double[] posSamples) {
            this.dist = distSamples;
            this.pos  = posSamples;
        }

        /** Returns interpolated hood position for the given distance (inches). */
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

    public static final HoodLUT HOOD_LUT = new HoodLUT(HOOD_DISTANCE_SAMPLES, HOOD_POSITION_SAMPLES);

    /**
     * Returns the clamped hood position, applying the live offset.
     * @param distance distance to goal in inches
     * @param offset   live offset (gamepad2 tuning)
     */
    public static double hoodPosition(double distance, double offset) {
        double raw = HOOD_LUT.get(distance) + offset;
        return clamp(raw, HOOD_MIN_POSITION, HOOD_MAX_POSITION);
    }

    // ─────────────────────────────────────────────────────────────
    // STALL DETECTION (current polling)
    // ─────────────────────────────────────────────────────────────

    private int stallCheckCounter = 0;
    private boolean driveStalling = false;

    /** Call every loop. Returns true if any drive motor is drawing above the stall threshold. */
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

    // ─────────────────────────────────────────────────────────────
    // ALLIANCE SELECTION HELPERS
    // ─────────────────────────────────────────────────────────────

    public static Alliance selectAlliance(boolean triangle, boolean circle) {
        if (triangle) return Alliance.RED;
        if (circle)   return Alliance.BLUE;
        return ALLIANCE; // keep current
    }

    public static Point goalCoordsForAlliance(Alliance a) {
        return (a == Alliance.RED) ? RED_GOAL_COORDS : BLUE_GOAL_COORDS;
    }

    // ─────────────────────────────────────────────────────────────
    // GAMEPAD2 OFFSET HELPERS
    // ─────────────────────────────────────────────────────────────

    public void adjustFlywheelOffset(boolean dpadUp, boolean dpadDown) {
        if (dpadUp)   flywheelVelocityOffset += FLYWHEEL_VELOCITY_OFFSET_JUMP;
        if (dpadDown) flywheelVelocityOffset -= FLYWHEEL_VELOCITY_OFFSET_JUMP;
    }

    public void adjustHoodOffset(boolean dpadLeft, boolean dpadRight) {
        if (dpadLeft)  hoodAngleOffset += HOOD_ANGLE_OFFSET_JUMP;
        if (dpadRight) hoodAngleOffset -= HOOD_ANGLE_OFFSET_JUMP;
    }

    // ─────────────────────────────────────────────────────────────
    // CLEAR BULK CACHE
    // ─────────────────────────────────────────────────────────────

    public void clearBulkCache() {
        if (controlHub != null) controlHub.clearBulkCache();
    }
}
