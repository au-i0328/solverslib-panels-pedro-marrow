package org.firstinspires.ftc.teamcode.subsystems;

import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import org.firstinspires.ftc.teamcode.RobotHardware;

/**
 * Flywheel subsystem using SolversLib MotorEx's built-in velocity PIDF controller.
 * Each motor runs an independent PIDF loop configured via setVelocityCoefficients().
 *
 * To switch between this and FlywheelSubsystem (voltage-loop):
 *   Set FLYWHEEL_USE_VOLTAGE_LOOP = false in RobotHardware.java
 *
 * Control loop per motor (handled by MotorEx):
 *   motor.setVelocity(targetVelocity)
 *   MotorEx internally runs: output = kP·err + kI·∫err + kD·d(err)/dt + kF·targetVelocity
 *   output is scaled by battery voltage automatically.
 */
public class FlywheelSubsystemSimple implements com.seattlesolvers.solverslib.command.Subsystem {
    private final MotorEx motorL;
    private final MotorEx motorR;

    public double targetVelocity = RobotHardware.FLYWHEEL_TARGET_VELOCITY;

    public FlywheelSubsystemSimple(RobotHardware hw) {
        this.motorL = hw.flywheelL;
        this.motorR = hw.flywheelR;

        // MotorEx's built-in velocity PID is used for flywheel control.
        // setFeedforwardCoefficients(ks, kv): ks = static friction, kv = velocity constant.
        // KV is not defined in RobotHardware for flywheels, so use 0 (PID handles everything).
        motorL.setFeedforwardCoefficients(RobotHardware.FLYWHEEL_L_KF, 0.0);
        motorR.setFeedforwardCoefficients(RobotHardware.FLYWHEEL_R_KF, 0.0);
    }

    public void addOffset(double delta) {
        targetVelocity += delta;
    }

    public double getTargetVelocity() {
        return targetVelocity;
    }

    public double getVelocityL() { return motorL.getVelocity(); }
    public double getVelocityR() { return motorR.getVelocity(); }

    public boolean isAtTarget() {
        double tol = RobotHardware.FLYWHEEL_READY_TOLERANCE;
        return Math.abs(getVelocityL() - targetVelocity) < tol
            && Math.abs(getVelocityR() - targetVelocity) < tol;
    }

    /**
     * Command both flywheel motors to targetVelocity using their SolversLib PIDF loops.
     * Call every loop tick.
     *
     * MotorEx.setVelocity() applies the configured PIDF coefficients internally and
     * scales the output by battery voltage automatically — no manual voltage math needed.
     */
    public void update() {
        motorL.setVelocity(targetVelocity);
        motorR.setVelocity(targetVelocity);
    }

    /** @deprecated Use update() — this method is kept for compatibility. */
    @Deprecated
    public void update(double dt) {
        update();
    }

    public void stop() {
        motorL.stopMotor();
        motorR.stopMotor();
    }
}
