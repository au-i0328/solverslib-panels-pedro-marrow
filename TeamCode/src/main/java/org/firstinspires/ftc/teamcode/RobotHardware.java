package org.firstinspires.ftc.teamcode;

import static com.seattlesolvers.solverslib.util.MathUtils.clamp;

import androidx.annotation.NonNull;

import com.pedropathing.geometry.Point;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx.CurrentUnit;
import com.qualcomm.robotcore.util.Range;
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

    /** Goal position in Pedro coordinates (origin = bottom-left, [0,144]). */
    public static Point GOAL_COORDS = new Point(144, 72);

    public static Point RED_GOAL_COORDS = new Point(144, 72);
    public static Point BLUE_GOAL_COORDS = new Point(0, 72);

    /** Base zone center positions. */
    public static Point RED_BASE_COORDS = new Point(120, 20);
    public static Point BLUE_BASE_COORDS = new Point(24, 20);

    /** Launch zone boundary line: 5 cm (≈2 inches) inboard from the scoring wall. */
    public static Point LAUNCH_ZONE_NEAR_LEFT = new Point(142, 72);
    public static Point LAUNCH_ZONE_NEAR_RIGHT = new Point(142, 72);
    // The actual 5 cm offset boundary will be computed relative to the scoring wall.

    // ─────────────────────────────────────────────────────────────
    // FLYWHEEL
    // ─────────────────────────────────────────────────────────────

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

    public static double DRIVE_STALL_CURRENT_THRESHOLD = 2.5; // amps — triggers torque limiting
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
    public static double VECTOR_WEIGHT_DRIVER = 0.6;

    // ─────────────────────────────────────────────────────────────
    // ODOMETRY INITIAL POSE
    // ─────────────────────────────────────────────────────────────

    public static double ODOM_INIT_X      = 72.0;
    public static double ODOM_INIT_Y      = 72.0;
    public static double ODOM_INIT_ANGLE  = 0.0; // radians

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

    public DcMotorEx fl, fr, bl, br;
    public DcMotorEx flywheelL, flywheelR;
    public DcMotorEx intake;
    public Servo hoodL, hoodR, gate;
    public IMU imu;
    public LynxModule controlHub;
    public VoltageSensor voltageSensor;

    // Panels TelemetryManager — set once per OpMode
    public org.bylazar.ftcontrol.panels.Panels panels;
    public com.seattlesolvers.solverslib.util.TelemetryData telemetryData;

    // Live flywheel velocity offset (modified by gamepad2 dpad)
    public double flywheelVelocityOffset = 0.0;

    // Live hood angle offset (modified by gamepad2 dpad)
    public double hoodAngleOffset = 0.0;

    // ─────────────────────────────────────────────────────────────
    // INIT
    // ─────────────────────────────────────────────────────────────

    public void init(@NonNull HardwareMap hwMap) {
        // Drive motors
        fl = hwMap.get(DcMotorEx.class, "FL");
        fr = hwMap.get(DcMotorEx.class, "FR");
        bl = hwMap.get(DcMotorEx.class, "BL");
        br = hwMap.get(DcMotorEx.class, "BR");

        fr.setDirection(DcMotor.Direction.REVERSE);
        br.setDirection(DcMotor.Direction.REVERSE);

        // Drivetrain motors: coast when power = 0 (so auto-align doesn't fight brake)
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Flywheel motors with encoders
        flywheelL = hwMap.get(DcMotorEx.class, "flywheelL");
        flywheelR = hwMap.get(DcMotorEx.class, "flywheelR");
        flywheelL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheelR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Intake motor
        intake = hwMap.get(DcMotorEx.class, "intake");

        // Servos
        hoodL  = hwMap.get(Servo.class, "hoodL");
        hoodR  = hwMap.get(Servo.class, "hoodR");
        gate   = hwMap.get(Servo.class, "gate");

        // Gate starts closed
        gate.setPosition(GATE_CLOSE_POSITION);

        // IMU — UP + LEFT orientation for Control Hub internal IMU
        // "UP" = logo pointing up; "LEFT" = USB port pointing left
        imu = hwMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT
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

    /**
     * Converts a target voltage to a motor power fraction, accounting for battery sag.
     * This is the foundation of torque-current control — always use this instead of
     * raw setPower().
     */
    public double voltageToPower(double targetVolts) {
        return Range.clip(targetVolts / batteryVoltage(), -1.0, 1.0);
    }

    // ─────────────────────────────────────────────────────────────
    // TORQUE-CURRENT CONTROL (drivetrain)
    // ─────────────────────────────────────────────────────────────

    // GoBilda 1177 / yellow-jacket constants
    private static final double TPR          = 28.0;        // ticks/rev
    private static final double I_STALL      = 9.2;         // amps
    private static final double R_MOTOR       = 12.0 / I_STALL; // ohms ≈ 1.30
    private static final double KEMF         = 0.0192;      // V/(rad/s)
    private static final double KTORQUE       = 0.0157;      // N-m/A

    private static final double MAX_CURRENT  = 3.0;         // amps per motor

    /**
     * Apply torque-current control to all four drive motors.
     * @param forward   normalized forward stick (-1 to 1)
     * @param strafe    normalized strafe stick (-1 to 1)
     * @param rotation  normalized rotation stick (-1 to 1)
     */
    public void setDrivePower(double forward, double strafe, double rotation) {
        double[] raw = mecanumPowers(forward, strafe, rotation);
        double batt = batteryVoltage();

        DcMotorEx[] motors = { fl, fr, bl, br };
        for (int i = 0; i < 4; i++) {
            double vRaw = raw[i] * 12.0; // voltage from joystick
            double omega = motors[i].getVelocity() / TPR * 2.0 * Math.PI; // rad/s
            double vBackEmf = KEMF * omega;

            double iTarget = Math.abs(raw[i]) < 0.01 ? 0.0 : MAX_CURRENT;
            double vMin = vBackEmf - iTarget * R_MOTOR;
            double vMax = vBackEmf + iTarget * R_MOTOR;
            double vCmd = Range.clip(vRaw, Math.min(vMin, vMax), Math.max(vMin, vMax));
            motors[i].setPower(Range.clip(vCmd / batt, -1.0, 1.0));
        }
    }

    /**
     * mecanum drive inverse kinematics.
     * Returns raw motor powers before current limiting.
     */
    public static double[] mecanumPowers(double fwd, double strafe, double rot) {
        double mag = Math.abs(fwd) + Math.abs(strafe) + Math.abs(rot);
        if (mag < 0.01) return new double[]{ 0, 0, 0, 0 };

        double norm = mag > 1.0 ? mag : 1.0;
        fwd    /= norm;
        strafe /= norm;
        rot    /= norm;

        double fl_pow = fwd + strafe + rot;
        double fr_pow = fwd - strafe - rot;
        double bl_pow = fwd - strafe + rot;
        double br_pow = fwd + strafe - rot;

        // re-normalize so max = 1
        double maxAbs = Math.max(
            Math.max(Math.abs(fl_pow), Math.abs(fr_pow)),
            Math.max(Math.abs(bl_pow), Math.abs(br_pow))
        );
        if (maxAbs > 1.0) {
            double inv = 1.0 / maxAbs;
            fl_pow *= inv; fr_pow *= inv;
            bl_pow *= inv; br_pow *= inv;
        }
        return new double[]{ fl_pow, fr_pow, bl_pow, br_pow };
    }

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
    // FLYWHEEL PIDF
    // ─────────────────────────────────────────────────────────────

    public static class FlywheelController {
        private final DcMotorEx motor;
        private final double kP, kI, kD, kF;
        private double integral = 0.0;

        public FlywheelController(DcMotorEx motor, double kP, double kI, double kD, double kF) {
            this.motor = motor;
            this.kP = kP; this.kI = kI; this.kD = kD; this.kF = kF;
        }

        public double update(double targetVelocity, double batteryVoltage) {
            double measured = motor.getVelocity();
            double error = targetVelocity - measured;
            integral += error * 0.001; // loop ≈ 1 ms
            integral = clamp(integral, -12.0, 12.0);
            double derivative = -motor.getVelocity(); // approximated from velocity change
            double power = kP * error + kI * integral + kD * derivative + kF * targetVelocity;
            return Range.clip(power / batteryVoltage, -1.0, 1.0);
        }

        public double getVelocity() { return motor.getVelocity(); }
        public void setPower(double power) { motor.setPower(power); }
        public void reset() { integral = 0.0; }
    }

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

    /** Call every loop. Returns true if any drive motor is currently stalling. */
    public boolean isDriveStallingAny() {
        stallCheckCounter++;
        boolean anyStalling =
            fl.getCurrent(CurrentUnit.AMPS) > DRIVE_STALL_CURRENT_THRESHOLD ||
            fr.getCurrent(CurrentUnit.AMPS) > DRIVE_STALL_CURRENT_THRESHOLD ||
            bl.getCurrent(CurrentUnit.AMPS) > DRIVE_STALL_CURRENT_THRESHOLD ||
            br.getCurrent(CurrentUnit.AMPS) > DRIVE_STALL_CURRENT_THRESHOLD;

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
        return a == Alliance.RED ? RED_GOAL_COORDS : BLUE_GOAL_COORDS;
    }

    public static Point baseCoordsForAlliance(Alliance a) {
        return a == Alliance.RED ? RED_BASE_COORDS : BLUE_BASE_COORDS;
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
