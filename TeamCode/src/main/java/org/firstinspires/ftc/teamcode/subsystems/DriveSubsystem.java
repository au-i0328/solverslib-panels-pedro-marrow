package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorEx.CurrentUnit;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.RobotHardware;

public class DriveSubsystem extends com.seattlesolvers.solverslib.command.Subsystem {
    private final DcMotorEx fl, fr, bl, br;
    private final Follower follower;

    // Torque-current control constants (GoBilda 1177)
    private static final double TPR        = 28.0;
    private static final double I_STALL    = 9.2;
    private static final double R_MOTOR    = 12.0 / I_STALL;
    private static final double KEMF       = 0.0192;
    private static final double MAX_CURRENT = 3.0;

    public DriveSubsystem(RobotHardware hw, Follower follower) {
        this.fl = hw.fl;
        this.fr = hw.fr;
        this.bl = hw.bl;
        this.br = hw.br;
        this.follower = follower;
    }

    /**
     * Field-centric mecanum drive with torque-current control and stall limiting.
     * Applies input curve, voltage compensation, and per-motor current clamping.
     */
    public void driveFieldCentric(double forward, double strafe, double rotation,
                                  double yawRadians, double batteryVoltage,
                                  boolean isStalling) {
        // Apply input curve
        forward  = RobotHardware.applyInputCurve(forward, RobotHardware.DRIVE_INPUT_CURVE_EXP);
        strafe   = RobotHardware.applyInputCurve(strafe,  RobotHardware.DRIVE_INPUT_CURVE_EXP);
        rotation = RobotHardware.applyInputCurve(rotation, RobotHardware.DRIVE_INPUT_CURVE_EXP);

        // Rotate joystick vectors by -yaw to get field-relative
        double cos = Math.cos(-yawRadians);
        double sin = Math.sin(-yawRadians);
        double fwdField = forward * cos - strafe * sin;
        double strafeField = forward * sin + strafe * cos;

        double[] raw = RobotHardware.mecanumPowers(fwdField, strafeField, rotation);
        DcMotorEx[] motors = { fl, fr, bl, br };
        double[] powers = new double[4];

        int checkInterval = isStalling ? 1 : RobotHardware.CURRENT_CHECK_INTERVAL;

        for (int i = 0; i < 4; i++) {
            double vRaw = raw[i] * 12.0;
            double omega = motors[i].getVelocity() / TPR * 2.0 * Math.PI;
            double vBackEmf = KEMF * omega;

            double iTarget = Math.abs(raw[i]) < 0.01 ? 0.0 : MAX_CURRENT;
            double vMin = vBackEmf - iTarget * R_MOTOR;
            double vMax = vBackEmf + iTarget * R_MOTOR;
            double vCmd = Range.clip(vRaw, Math.min(vMin, vMax), Math.max(vMin, vMax));

            powers[i] = Range.clip(vCmd / batteryVoltage, -1.0, 1.0);
        }

        fl.setPower(powers[0]);
        fr.setPower(powers[1]);
        bl.setPower(powers[2]);
        br.setPower(powers[3]);
    }

    /**
     * Robot-centric drive with same torque-current control.
     */
    public void driveRobotCentric(double forward, double strafe, double rotation,
                                  double batteryVoltage, boolean isStalling) {
        forward  = RobotHardware.applyInputCurve(forward,  RobotHardware.DRIVE_INPUT_CURVE_EXP);
        strafe   = RobotHardware.applyInputCurve(strafe,   RobotHardware.DRIVE_INPUT_CURVE_EXP);
        rotation = RobotHardware.applyInputCurve(rotation, RobotHardware.DRIVE_INPUT_CURVE_EXP);

        double[] raw = RobotHardware.mecanumPowers(forward, strafe, rotation);
        DcMotorEx[] motors = { fl, fr, bl, br };

        for (int i = 0; i < 4; i++) {
            double vRaw = raw[i] * 12.0;
            double omega = motors[i].getVelocity() / TPR * 2.0 * Math.PI;
            double vBackEmf = KEMF * omega;

            double iTarget = Math.abs(raw[i]) < 0.01 ? 0.0 : MAX_CURRENT;
            double vMin = vBackEmf - iTarget * R_MOTOR;
            double vMax = vBackEmf + iTarget * R_MOTOR;
            double vCmd = Range.clip(vRaw, Math.min(vMin, vMax), Math.max(vMin, vMax));

            motors[i].setPower(Range.clip(vCmd / batteryVoltage, -1.0, 1.0));
        }
    }

    public void stop() {
        fl.setPower(0); fr.setPower(0);
        bl.setPower(0); br.setPower(0);
    }

    public boolean isAnyStalling() {
        return fl.getCurrent(CurrentUnit.AMPS) > RobotHardware.DRIVE_STALL_CURRENT_THRESHOLD
            || fr.getCurrent(CurrentUnit.AMPS) > RobotHardware.DRIVE_STALL_CURRENT_THRESHOLD
            || bl.getCurrent(CurrentUnit.AMPS) > RobotHardware.DRIVE_STALL_CURRENT_THRESHOLD
            || br.getCurrent(CurrentUnit.AMPS) > RobotHardware.DRIVE_STALL_CURRENT_THRESHOLD;
    }
}
