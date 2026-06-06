package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.follower.Follower;
import com.seattlesolvers.solverslib.drivebase.MecanumDrive;
import com.seattlesolvers.solverslib.hardware.MotorEx;
import com.seattlesolvers.solverslib.hardware.Motor;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.controller.Feedforward;
import com.seattlesolvers.solverslib.util.SimpleMotorFeedforward;

/**
 * Drivetrain subsystem using SolversLib MecanumDrive.
 *
 * MecanumDrive handles field-centric / robot-centric kinematics internally.
 * A PIDFController corrects heading error during auto-align; a SimpleMotorFeedforward
 * provides kS/kV/kA feedforward so the PID loop only handles disturbance correction.
 */
public class DriveSubsystem extends com.seattlesolvers.solverslib.command.Subsystem {
    private final MotorEx fl, fr, bl, br;
    private final MecanumDrive mecanum;
    private final Follower follower;

    // Heading PIDF for auto-align rotation correction
    private static final double HEADING_KP = 0.05;
    private static final double HEADING_KI = 0.0;
    private static final double HEADING_KD = 0.0;
    private static final double HEADING_KF = 0.0;
    private final PIDFController headingPid = new PIDFController(HEADING_KP, HEADING_KI, HEADING_KD, HEADING_KF);

    // Feedforward: kS = static friction, kV = velocity, kA = acceleration
    // Tune via Panels by changing DRIVE_KFF_S / DRIVE_KFF_V / DRIVE_KFF_A
    public static double DRIVE_KFF_S = 0.0;
    public static double DRIVE_KFF_V = 1.0 / 2800.0;  // fraction of max speed per ticks/sec
    public static double DRIVE_KFF_A = 0.0;
    private final SimpleMotorFeedforward feedforward =
        new SimpleMotorFeedforward(DRIVE_KFF_S, DRIVE_KFF_V, DRIVE_KFF_A);

    public DriveSubsystem(RobotHardware hw, Follower follower) {
        this.fl = hw.fl;
        this.fr = hw.fr;
        this.bl = hw.bl;
        this.br = hw.br;
        this.follower = follower;

        this.mecanum = new MecanumDrive(fl, fr, bl, br);
        this.mecanum.setFeedforward(feedforward);
    }

    /**
     * Field-centric mecanum drive.
     * @param forward   forward/back (positive = forward)
     * @param strafe    left/right (positive = right)
     * @param rotation  rotation (positive = CW)
     * @param yawRadians current IMU yaw in radians
     */
    public void driveFieldCentric(double forward, double strafe, double rotation, double yawRadians) {
        mecanum.driveFieldCentric(strafe, forward, rotation, Math.toDegrees(yawRadians));
    }

    /**
     * Robot-centric mecanum drive.
     */
    public void driveRobotCentric(double forward, double strafe, double rotation) {
        mecanum.driveRobotCentric(strafe, forward, rotation);
    }

    /**
     * Drive with heading correction (for auto-align to limelight crosshairs).
     * @param forward  forward/back
     * @param strafe   left/right
     * @param rotation driver rotation override (added to heading correction)
     * @param yawRadians current IMU yaw in radians
     * @param targetHeadingDegrees heading to hold (from odometry or limelight)
     */
    public void driveWithHeadingCorrection(double forward, double strafe, double rotation,
                                          double yawRadians, double targetHeadingDegrees) {
        // Feedforward correction toward target heading
        headingPid.setSetPoint(targetHeadingDegrees);
        double currentHeadingDeg = Math.toDegrees(yawRadians);
        double correction = headingPid.calculate(currentHeadingDeg);

        double totalRotation = rotation + correction;
        mecanum.driveFieldCentric(strafe, forward, totalRotation, currentHeadingDeg);
    }

    public void stop() {
        mecanum.stop();
    }
}
