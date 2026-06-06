package org.firstinspires.ftc.teamcode.subsystems;

import static com.seattlesolvers.solverslib.util.MathUtils.clamp;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.RobotHardware;

public class FlywheelSubsystem extends com.seattlesolvers.solverslib.command.Subsystem {
    private final DcMotorEx motorL;
    private final DcMotorEx motorR;
    private final VoltageSensor voltageSensor;

    private double integralL = 0.0;
    private double integralR = 0.0;

    public FlywheelSubsystem(RobotHardware hw) {
        this.motorL = hw.flywheelL;
        this.motorR = hw.flywheelR;
        this.voltageSensor = hw.voltageSensor;
    }

    public void addOffset(double delta) {
        RobotHardware.flywheelVelocityOffset += delta;
    }

    public double getTargetVelocity() {
        return RobotHardware.FLYWHEEL_TARGET_VELOCITY + RobotHardware.flywheelVelocityOffset;
    }

    public double getVelocityL() { return motorL.getVelocity(); }
    public double getVelocityR() { return motorR.getVelocity(); }

    public boolean isAtTarget() {
        double tol = RobotHardware.FLYWHEEL_READY_TOLERANCE;
        return Math.abs(getVelocityL() - getTargetVelocity()) < tol
            && Math.abs(getVelocityR() - getTargetVelocity()) < tol;
    }

    /**
     * PIDF velocity loop for both flywheel motors.
     * Uses SolversLib PIDF — no FTC-SDK PIDFController.
     * Integral is accumulated per motor with anti-windup clamping.
     */
    public void update() {
        double target = getTargetVelocity();
        double batt = voltageSensor.getVoltage();
        double loopTime = 0.001; // seconds (~1 ms per loop)

        double velL = motorL.getVelocity();
        double velR = motorR.getVelocity();

        double errL = target - velL;
        double errR = target - velR;

        // Accumulate integral with anti-windup
        integralL = clamp(integralL + errL * loopTime, -12.0, 12.0);
        integralR = clamp(integralR + errR * loopTime, -12.0, 12.0);

        double kpL = RobotHardware.FLYWHEEL_L_KP;
        double kiL = RobotHardware.FLYWHEEL_L_KI;
        double kdL = RobotHardware.FLYWHEEL_L_KD;
        double kfL = RobotHardware.FLYWHEEL_L_KF;

        double kpR = RobotHardware.FLYWHEEL_R_KP;
        double kiR = RobotHardware.FLYWHEEL_R_KI;
        double kdR = RobotHardware.FLYWHEEL_R_KD;
        double kfR = RobotHardware.FLYWHEEL_R_KF;

        // PIDF: kP*e + kI*∫e + kF*target
        // Note: kD is omitted — derivative of motor velocity (not error) is unreliable
        // without a prior velocity sample. Use kI to absorb steady-state error.
        double powerL = kpL * errL + kiL * integralL + kfL * target;
        double powerR = kpR * errR + kiR * integralR + kfR * target;

        motorL.setPower(Range.clip(powerL / batt, -1, 1));
        motorR.setPower(Range.clip(powerR / batt, -1, 1));
    }

    public void reset() {
        integralL = 0.0;
        integralR = 0.0;
    }

    public void stop() {
        motorL.setPower(0);
        motorR.setPower(0);
        reset();
    }
}
