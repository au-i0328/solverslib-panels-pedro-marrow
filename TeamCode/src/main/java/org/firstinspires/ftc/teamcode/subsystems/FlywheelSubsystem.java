package org.firstinspires.ftc.teamcode.subsystems;

import com.seattlesolvers.solverslib.hardware.MotorEx;

/**
 * Flywheel subsystem using SolversLib MotorEx in VelocityControl.
 *
 * Voltage loop per motor (per cycle):
 *
 *   ω_measured  = motor.getVelocity() / TPR × 2π          (rad/s from ticks/s)
 *   vBackEmf   = K_EMF × ω_measured                     (opposes applied voltage)
 *
 *   PID corrective voltage:
 *     err        = targetVelocity − motor.getVelocity()
 *     integral   = clamp(integral + err·dt, −MAX_INT_VOLTAGE/ki, MAX_INT_VOLTAGE/ki)
 *     derivative = (dt > 0.0001) ? (err − prevErr) / dt : 0
 *     vPID      = kP·err + kI·integral + kD·derivative + kF·targetVelocity
 *
 *   vTarget    = vPID + vBackEmf                         (back-EMF compensation)
 *   power      = vTarget / batteryVoltage
 *
 * No current limiting is applied to the flywheel motors — current limiting
 * is only used on the drive motors. The integral clamp (MAX_INTEGRAL_VOLTAGE)
 * prevents wind-up without needing a current ceiling.
 */
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
     * Voltage-based closed-loop velocity update — call every loop.
     *
     * @param dt loop time in seconds (pass loopTimer.seconds() from the OpMode)
     */
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
        double maxIntSum = (ki == 0.0) ? 0.0 : (RobotHardware.FLYWHEEL_MAX_INTEGRAL_VOLTAGE / ki);
        integral = clamp(integral + err * dt, -maxIntSum, maxIntSum);

        // Guard against division by zero on first loop or stalls
        double deriv = (dt > 0.0001) ? (err - prevErr) / dt : 0.0;

        double vPID    = kp * err + ki * integral + kd * deriv + kf * targetVel;
        double vTarget = vPID + vBackEmf;

        motor.set(vTarget / batt);

        // Persist state
        if (motor == motorL) {
            integralL = integral; prevErrL = err;
        } else {
            integralR = integral; prevErrR = err;
        }
    }

    /** Legacy zero-argument update — assumes 1 ms loop. Prefer update(double dt). */
    public void update() {
        update(0.001);
    }

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
