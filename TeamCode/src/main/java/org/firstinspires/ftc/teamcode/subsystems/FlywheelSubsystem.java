package org.firstinspires.ftc.teamcode.subsystems;

import com.seattlesolvers.solverslib.hardware.MotorEx;
import com.seattlesolvers.solverslib.hardware.Motor;
import com.seattlesolvers.solverslib.util.SimpleMotorFeedforward;

/**
 * Flywheel subsystem using SolversLib MotorEx in VelocityControl.
 *
 * Closed-loop velocity is implemented as:
 *   power = (kP*e + kI*∫e + kD*de/dt + kF*targetVelocity + kS*sign + kV*targetVel + kA*accel) / batteryVoltage
 *
 * The PIDF terms and feedforward are computed manually for clarity and correctness.
 * SimpleMotorFeedforward provides kS/kV/kA without the kF term (kF is in the PID sum).
 */
public class FlywheelSubsystem extends com.seattlesolvers.solverslib.command.Subsystem {
    private final MotorEx motorL;
    private final MotorEx motorR;

    // Velocity targets in ticks/sec — mutable so gamepad offsets can adjust them
    public double targetVelocity = RobotHardware.FLYWHEEL_TARGET_VELOCITY;

    // Per-motor PIDF state
    private double integralL = 0.0;
    private double integralR = 0.0;
    private double prevErrL = 0.0;
    private double prevErrR = 0.0;

    public FlywheelSubsystem(RobotHardware hw) {
        this.motorL = hw.flywheelL;
        this.motorR = hw.flywheelR;
        // Motors are in VelocityControl mode from RobotHardware.init()
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
     * Closed-loop velocity update — call every loop.
     *
     * Loop equation (per motor):
     *   voltage = kP*e + kI*∫e + kD*de/dt + kF*target + feedforward(target)
     *   power   = voltage / batteryVoltage
     *
     * @param dt loop time in seconds (pass loopTimer.seconds() from the OpMode)
     */
    public void update(double dt) {
        double velL = motorL.getVelocity();
        double velR = motorR.getVelocity();
        double batt = RobotHardware.batteryVoltage();

        // PIDF error signals
        double errL = targetVelocity - velL;
        double errR = targetVelocity - velR;

        // Accumulate integral with anti-windup (clamp to ±12 V range)
        integralL = clamp(integralL + errL * dt, -12.0, 12.0);
        integralR = clamp(integralR + errR * dt, -12.0, 12.0);

        // Derivative: de/dt
        double derivL = (errL - prevErrL) / dt;
        double derivR = (errR - prevErrR) / dt;
        prevErrL = errL;
        prevErrR = errR;

        // PIDF terms (kF is the feedforward velocity constant, separate from SimpleMotorFeedforward)
        double kpL = RobotHardware.FLYWHEEL_L_KP;
        double kiL = RobotHardware.FLYWHEEL_L_KI;
        double kdL = RobotHardware.FLYWHEEL_L_KD;
        double kfL = RobotHardware.FLYWHEEL_L_KF;

        double kpR = RobotHardware.FLYWHEEL_R_KP;
        double kiR = RobotHardware.FLYWHEEL_R_KI;
        double kdR = RobotHardware.FLYWHEEL_R_KD;
        double kfR = RobotHardware.FLYWHEEL_R_KF;

        // PIDF: kP*e + kI*∫e + kD*de/dt + kF*targetVelocity
        double voltageL = kpL * errL + kiL * integralL + kdL * derivL + kfL * targetVelocity;
        double voltageR = kpR * errR + kiR * integralR + kdR * derivR + kfR * targetVelocity;

        // SolversLib SimpleMotorFeedforward: voltage = kS*sign(v) + kV*velocity + kA*accel
        // kS ≈ 0.15 V (overcomes static friction at ~0.125 A × 1.2 Ω)
        // kV = 12 V / maxVelocity → set(1.0) = maxVelocity
        double maxVel = RobotHardware.FLYWHEEL_TARGET_VELOCITY;
        double kSL = 0.15;
        double kVL = 12.0 / maxVel;
        double kSR = 0.15;
        double kVR = 12.0 / maxVel;

        double ffL = kSL * Math.signum(targetVelocity) + kVL * targetVelocity;
        double ffR = kSR * Math.signum(targetVelocity) + kVR * targetVelocity;

        // Total voltage → motor power fraction
        motorL.set((voltageL + ffL) / batt);
        motorR.set((voltageR + ffR) / batt);
    }

    /** Legacy zero-argument update — assumes 1 ms loop. Prefer update(double dt). */
    public void update() {
        update(0.001);
    }

    public void reset() {
        integralL = 0.0;
        integralR = 0.0;
        prevErrL = 0.0;
        prevErrR = 0.0;
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
