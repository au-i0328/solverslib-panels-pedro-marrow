package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.TwoWheelConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

/**
 * Pedro Pathing constants — all fields are {@code public static} so Panels
 * Configurables can live-tune them without redeploying code.
 *
 * Tuning order:
 *   1. Localization  → TwoWheelConstants (encoder names, IMU, ticks-to-inches multipliers)
 *   2. Follower mass → FollowerConstants.mass
 *   3. Velocity      → Forward Velocity Tuner, Lateral Velocity Tuner
 *   4. ZPA           → Forward / Lateral Zero Power Acceleration Tuner
 *   5. Heading       → Heading Tuner
 *   6. Translational → Translational Tuner
 *   7. Drive         → Drive Tuner
 *   8. Centripetal   → Centripetal Tuner
 *
 * Hardware naming — match these to the names in your Robot Controller
 * configuration XML. The odometry pod names ("forwardEncoder", "strafeEncoder")
 * must refer to motors plugged into the two fastest encoder ports (0 and 3 on
 * REV Control Hub) for best odometry performance.
 */
@com.bylazar.configurables.annotations.Configurable
public class Constants {

    // ═══════════════════════════════════════════════════════════════════════
    // DRIVE MOTOR NAMES
    // ═══════════════════════════════════════════════════════════════════════
    // Must match the names in the Robot Controller configuration XML.

    public static String LEFT_FRONT_MOTOR  = "FL";
    public static String LEFT_REAR_MOTOR   = "BL";
    public static String RIGHT_FRONT_MOTOR = "FR";
    public static String RIGHT_REAR_MOTOR  = "BR";

    // ═══════════════════════════════════════════════════════════════════════
    // DRIVE MOTOR DIRECTIONS
    // ═══════════════════════════════════════════════════════════════════════
    // If the robot drives in the wrong direction, flip the affected motor here.
    // Run "Localization Test" to check: pushing the left stick forward should
    // increase X; pushing it left should increase Y.

    public static DcMotorSimple.Direction LEFT_FRONT_DIRECTION  = DcMotorSimple.Direction.FORWARD;
    public static DcMotorSimple.Direction LEFT_REAR_DIRECTION   = DcMotorSimple.Direction.FORWARD;
    public static DcMotorSimple.Direction RIGHT_FRONT_DIRECTION = DcMotorSimple.Direction.REVERSE;
    public static DcMotorSimple.Direction RIGHT_REAR_DIRECTION  = DcMotorSimple.Direction.REVERSE;

    // ═══════════════════════════════════════════════════════════════════════
    // ODOMETRY POD NAMES
    // ═══════════════════════════════════════════════════════════════════════
    // forwardEncoder  — pod parallel to the robot's forward direction
    //                  (typically on the left or front-left motor)
    // strafeEncoder   — pod perpendicular to forward (right-encoder on right side)
    //
    // Recommended ports: encoder ports 0 (fastest) and 3 on REV Control Hub.
    // Using motor ports for encoders is preferred for speed.

    public static String FORWARD_ENCODER_HWMAP_NAME = "FL";    // parallel pod — forward encoder
    public static String STRAFE_ENCODER_HWMAP_NAME  = "BR"; // perpendicular pod — lateral encoder

    // ═══════════════════════════════════════════════════════════════════════
    // ODOMETRY POD DIRECTIONS
    // ═══════════════════════════════════════════════════════════════════════
    // Run "Forward Tuner": push robot forward 48 in, X should increase.
    // Run "Lateral Tuner": push robot left 48 in, Y should increase.
    // If an axis goes the wrong way, set the corresponding direction to REVERSE.
    // Values set here are applied via TwoWheelConstants.
    // 1.0 = FORWARD, -1.0 = REVERSE (matching TwoWheelConstants convention)

    public static double FORWARD_ENCODER_DIRECTION  = 1.0;   // 1.0 = FORWARD
    public static double STRAFE_ENCODER_DIRECTION   = 1.0;   // 1.0 = FORWARD

    // ═══════════════════════════════════════════════════════════════════════
    // ODOMETRY TICKS-TO-INCHES MULTIPLIERS
    // ═══════════════════════════════════════════════════════════════════════
    // Populated by the Forward / Lateral Tuners.
    //   Forward multiplier: 48 in / measured_forward_ticks
    //   Lateral  multiplier: 48 in / measured_lateral_ticks

    public static double FORWARD_TICKS_TO_INCHES  = 1.0;
    public static double STRAFE_TICKS_TO_INCHES  = 1.0;

    // ═══════════════════════════════════════════════════════════════════════
    // ODOMETRY POD OFFSETS (inches from robot center of rotation)
    // ═══════════════════════════════════════════════════════════════════════
    // Measured from the robot's center of rotation:
    //   forwardPodY — how far forward the forward pod is (positive = forward)
    //   strafePodX  — how far right the lateral pod is (positive = right)
    //
    // Run "Offsets Tuner" in Tuning → Localization → Offsets Tuner to
    // find these values automatically.

    public static double FORWARD_POD_Y  = 0.0;  // inches — forward pod offset in Y
    public static double STRAFE_POD_X  = 0.0;  // inches — lateral pod offset in X

    // ═══════════════════════════════════════════════════════════════════════
    // IMU
    // ═══════════════════════════════════════════════════════════════════════
    // Must match the IMU name in the Robot Controller configuration.
    // IMU orientation must match the physical mounting on the Control Hub.

    public static String IMU_HWMAP_NAME = "imu";

    /** IMU logo facing direction — match physical orientation on your Control Hub. */
    public static RevHubOrientationOnRobot.LogoFacingDirection IMU_LOGO_FACING =
            RevHubOrientationOnRobot.LogoFacingDirection.UP;

    /** IMU USB port facing direction — match physical orientation on your Control Hub. */
    public static RevHubOrientationOnRobot.UsbFacingDirection IMU_USB_FACING =
            RevHubOrientationOnRobot.UsbFacingDirection.LEFT;

    // ═══════════════════════════════════════════════════════════════════════
    // MAX POWER
    // ═══════════════════════════════════════════════════════════════════════
    // Overall speed cap applied to the drivetrain during path following.
    // 1.0 = full speed, 0.5 = half speed. Live-tunable via Panels.

    public static double MAX_POWER = 1.0;

    // ═══════════════════════════════════════════════════════════════════════
    // PATH CONSTRAINTS
    // ═══════════════════════════════════════════════════════════════════════
    // (maxVel, maxAccel, maxJerk, maxAngVel, maxAngAccel)

    public static double PATH_MAX_VELOCITY     = 40.0;   // in/s — max path following speed
    public static double PATH_MAX_ACCELERATION = 60.0;   // in/s²
    public static double PATH_MAX_JERK        = 120.0;  // in/s³
    public static double PATH_MAX_ANG_VELOCITY    = Math.toRadians(180); // rad/s
    public static double PATH_MAX_ANG_ACCELERATION = Math.toRadians(360); // rad/s²

    // ═══════════════════════════════════════════════════════════════════════
    // ROBOT MASS (kg)
    // ═══════════════════════════════════════════════════════════════════════
    // Used for centripetal force compensation. Must be in kilograms.
    // Weigh the robot, or weigh yourself holding the robot and subtract your weight.

    public static double ROBOT_MASS_KG = 5.0;

    // ═══════════════════════════════════════════════════════════════════════
    // VELOCITY MULTIPLIERS (in/s)
    // ═══════════════════════════════════════════════════════════════════════
    // Populated by the Forward / Lateral Velocity Tuners.
    // The follower uses these to scale commanded wheel velocities to real-world
    // inches-per-second for accurate path deceleration and braking calculations.

    public static double X_VELOCITY = 1.0;   // forward velocity multiplier (in/s)
    public static double Y_VELOCITY = 1.0;   // lateral velocity multiplier (in/s)

    // ═══════════════════════════════════════════════════════════════════════
    // ZERO-POWER ACCELERATION (in/s²)
    // ═══════════════════════════════════════════════════════════════════════
    // Populated by the Forward / Lateral Zero Power Acceleration Tuners.
    // Describes how quickly the robot decelerates when power is cut to the
    // drivetrain. Used internally for accurate path-end braking calculations.

    public static double FORWARD_ZPA = 50.0;   // in/s² — natural deceleration forward
    public static double LATERAL_ZPA = 50.0;   // in/s² — natural deceleration lateral

    // ═══════════════════════════════════════════════════════════════════════
    // CENTRIPETAL SCALING
    // ═══════════════════════════════════════════════════════════════════════
    // Populated by the Centripetal Tuner.
    // 0 = no centripetal correction. Typical tuned values: 0.001 – 0.01.
    //   Too low → robot under-corrects, drifts outside curves.
    //   Too high → robot over-corrects, oscillates on curves.

    public static double CENTRIPETAL_SCALING = 0.005;

    // ═══════════════════════════════════════════════════════════════════════
    // HEADING PIDF
    // ═══════════════════════════════════════════════════════════════════════
    // Populated by the Heading Tuner.
    // P — how aggressively the robot corrects heading errors
    // I — integral — eliminates steady-state heading drift
    // D — derivative — dampens oscillations
    // F — feedforward — baseline power to overcome friction / motor dead-zone

    public static double HEADING_P = 3.0;
    public static double HEADING_I = 0.0;
    public static double HEADING_D = 0.1;
    public static double HEADING_F = 0.0;

    // ═══════════════════════════════════════════════════════════════════════
    // TRANSLATIONAL PIDF
    // ═══════════════════════════════════════════════════════════════════════
    // Populated by the Translational Tuner.
    // Corrects lateral deviation from the desired path.

    public static double TRANSLATIONAL_P = 1.5;
    public static double TRANSLATIONAL_I = 0.0;
    public static double TRANSLATIONAL_D = 0.05;
    public static double TRANSLATIONAL_F = 0.0;

    // ═══════════════════════════════════════════════════════════════════════
    // DRIVE PIDF
    // ═══════════════════════════════════════════════════════════════════════
    // Populated by the Drive Tuner.
    // Manages acceleration and braking along the path.
    //
    // T (fourth parameter) — derivative filter time constant.
    //   Lower T (e.g. 0.2) = more responsive, noisier derivative.
    //   Higher T (e.g. 0.8) = smoother, more damped derivative.

    public static double DRIVE_P = 0.1;
    public static double DRIVE_I = 0.0;
    public static double DRIVE_D = 0.01;
    public static double DRIVE_F = 0.0;
    public static double DRIVE_T = 0.6;  // derivative filter time constant

    // ═══════════════════════════════════════════════════════════════════════
    // BRAKING STRENGTH (0 – 1)
    // ═══════════════════════════════════════════════════════════════════════
    // Scales how aggressively the drive PIDF brakes near the end of a path.
    // 1.0 = maximum braking, 0.5 = half braking. Live-tunable.

    public static double BRAKING_STRENGTH = 0.5;

    // ═══════════════════════════════════════════════════════════════════════
    // SECONDARY PIDFs (dual PID system — optional)
    // ═══════════════════════════════════════════════════════════════════════
    // Enable with .useSecondary*PIDF(true) in FollowerConstants below.
    // The secondary PID handles smaller residual errors after the primary PID
    // has brought the robot close to the path.
    //
    // Secondary Heading PIDF
    public static double HEADING_SECONDARY_P = 1.5;
    public static double HEADING_SECONDARY_I = 0.0;
    public static double HEADING_SECONDARY_D = 0.05;
    public static double HEADING_SECONDARY_F = 0.0;

    // Secondary Translational PIDF
    public static double TRANSLATIONAL_SECONDARY_P = 0.75;
    public static double TRANSLATIONAL_SECONDARY_I = 0.0;
    public static double TRANSLATIONAL_SECONDARY_D = 0.02;
    public static double TRANSLATIONAL_SECONDARY_F = 0.0;

    // Secondary Drive PIDF
    public static double DRIVE_SECONDARY_P = 0.05;
    public static double DRIVE_SECONDARY_I = 0.0;
    public static double DRIVE_SECONDARY_D = 0.005;
    public static double DRIVE_SECONDARY_F = 0.0;
    public static double DRIVE_SECONDARY_T = 0.6;

    // ═══════════════════════════════════════════════════════════════════════
    // KALMAN FILTER — DRIVE PID
    // ═══════════════════════════════════════════════════════════════════════
    // Applied to the drive PID error signal to smooth out motor noise.
    // Higher model covariance → filter trusts previous output more (smoother).
    // Higher data covariance   → filter trusts raw error more (more responsive).

    public static double DRIVE_KALMAN_MODEL_COVARIANCE = 6.0;
    public static double DRIVE_KALMAN_DATA_COVARIANCE = 1.0;

    // ═══════════════════════════════════════════════════════════════════════
    // BUILT OBJECTS — rebuilt whenever a tunable constant changes
    // ═══════════════════════════════════════════════════════════════════════

    public static TwoWheelConstants localizerConstants;
    public static FollowerConstants followerConstants;
    public static PathConstraints pathConstraints;

    static {
        rebuild();
    }

    /** Call this whenever any constant above changes (Panels live-tune callback). */
    public static void rebuild() {
        localizerConstants =
            new TwoWheelConstants()
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
                .forwardZeroPowerAcceleration(FORWARD_ZPA)
                .lateralZeroPowerAcceleration(LATERAL_ZPA)
                .centripetalScaling(CENTRIPETAL_SCALING)
                .headingPIDFCoefficients(new com.pedropathing.control.PIDFCoefficients(
                        HEADING_P, HEADING_I, HEADING_D, HEADING_F))
                .translationalPIDFCoefficients(new com.pedropathing.control.PIDFCoefficients(
                        TRANSLATIONAL_P, TRANSLATIONAL_I, TRANSLATIONAL_D, TRANSLATIONAL_F))
                .drivePIDFCoefficients(new com.pedropathing.control.FilteredPIDFCoefficients(
                        DRIVE_P, DRIVE_I, DRIVE_D, DRIVE_T, DRIVE_F));

        pathConstraints =
            new PathConstraints(
                PATH_MAX_VELOCITY,
                PATH_MAX_ACCELERATION,
                PATH_MAX_JERK,
                PATH_MAX_ANG_VELOCITY
            );
    }

    /**
     * Creates a fully-configured Pedro Follower.
     * Call once per OpMode, typically in {@code init()} or {@code init_loop()}.
     *
     * The returned Follower uses:
     *   - TwoWheelLocalizer  (forward + lateral odometry pods + IMU)
     *   - Mecanum drivetrain (FL / BL / FR / BR motors)
     *   - All constants defined in this class — live-tunable via Panels
     */
    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .twoWheelLocalizer(localizerConstants)
                .mecanumDrivetrain(
                    new MecanumConstants()
                        .maxPower(MAX_POWER)
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
