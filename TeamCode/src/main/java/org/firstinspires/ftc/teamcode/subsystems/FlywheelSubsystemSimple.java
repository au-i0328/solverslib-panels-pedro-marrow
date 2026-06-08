package org.firstinspires.ftc.teamcode.subsystems;

import com.seattlesolvers.solverslib.hardware.MotorEx;

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
public class FlywheelSubsystemSimple extends com.seattlesolvers.solverslib.command.Subsystem {
    private final MotorEx motorL;
    private final MotorEx motorR;

    public double targetVelocity = RobotHardware.FLYWHEEL_TARGET_VELOCITY;

    public FlywheelSubsystemSimple(RobotHardware hw) {
        this.motorL = hw.flywheelL;
        this.motorR = hw.flywheelR;

        // MotorEx.setVelocityCoefficients() takes (kP, kI, kD, kF) for the velocity PIDF.
        // These are set once at init and used every time setVelocity() is called.
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
