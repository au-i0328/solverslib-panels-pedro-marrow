package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx.CurrentUnit;
import com.seattlesolvers.solverslib.hardware.MotorEx;

/**
 * Flywheel subsystem using SolversLib MotorEx in VelocityControl.
 *
 * Voltage loop per motor (per cycle):
 *
 *   ω_measured  = motor.getVelocity() / TPR × 2π         (rad/s from ticks/s)
 *   vBackEmf    = K_EMF × ω_measured                    (opposes applied voltage)
 *
 *   PID corrective voltage:
 *     err        = targetVelocity − ω_measured
 *     integral   = clamp(integral + err·dt, −12, 12)
 *     derivative = (err − prevErr) / dt
 *     vPID      = kP·err + kI·integral + kD·derivative + kF·targetVelocity
 *
 *   vTarget     = vPID + vBackEmf                        (back-EMF compensation)
 *
 *   Torque limiting (clamp voltage within what produces MAX_CURRENT):
 *     iTarget    = MAX_CURRENT  (always active — flywheels are load-bearing)
 *     vMin       = vBackEmf − iTarget × R_MOTOR
 *     vMax       = vBackEmf + iTarget × R_MOTOR
 *     vClamped   = clamp(vTarget, vMin, vMax)
 *
 *   power       = vClamped / batteryVoltage
 */
public class FlywheelSubsystem extends com.seattlesolvers.solverslib.command.Subsystem {
    private final MotorEx motorL;
    private final MotorEx motorR;

    // Velocity target in ticks/sec — mutable so gamepad offsets adjust it live
    public double targetVelocity = RobotHardware.FLYWHEEL_TARGET_VELOCITY;

    // Per-motor PIDF state
    private double integralL = 0.0;
    private double integralR = 0.0;
    private double prevErrL  = 0.0;
    private double prevErrR  = 0.0;

    public FlywheelSubsystem(RobotHardware hw) {
        this.motorL = hw.flywheelL;
        this.motorR = hw.flywheelR;
        // VelocityControl mode and feedforward are set in RobotHardware.init()
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
                RobotHardware.FLYWHEEL_R,
                RobotHardware.FLYWHEEL_MAX_CURRENT,
                RobotHardware.FLYWHEEL_L_KP,
                RobotHardware.FLYWHEEL_L_KI,
                RobotHardware.FLYWHEEL_L_KD,
                RobotHardware.FLYWHEEL_L_KF,
                new double[]{integralL, prevErrL});

        applyVoltageLoop(motorR, targetVelocity, dt, batt,
                RobotHardware.FLYWHEEL_TPR,
                RobotHardware.FLYWHEEL_K_EMF,
                RobotHardware.FLYWHEEL_R,
                RobotHardware.FLYWHEEL_MAX_CURRENT,
                RobotHardware.FLYWHEEL_R_KP,
                RobotHardware.FLYWHEEL_R_KI,
                RobotHardware.FLYWHEEL_R_KD,
                RobotHardware.FLYWHEEL_R_KF,
                new double[]{integralR, prevErrR});
    }

    /**
     * Voltage loop for a single motor.
     *
     * Arrays are used for integral/prevErr so the caller can persist state between loops
     * without exposing mutable fields.
     */
    private void applyVoltageLoop(
            MotorEx motor,
            double targetVel,   // ticks/s
            double dt,
            double batt,
            double tpr,
            double kEmf,
            double rMotor,
            double maxCurrent,
            double kp, double ki, double kd, double kf,
            double[] state     // [0] = integral, [1] = prevErr (mutated in place)
    ) {
        double omegaMeas  = motor.getVelocity() / tpr * 2.0 * Math.PI; // rad/s
        double vBackEmf  = kEmf * omegaMeas;

        // PID on velocity error
        double err = targetVel - motor.getVelocity(); // ticks/s error
        state[0] = clamp(state[0] + err * dt, -12.0, 12.0);
        double deriv = (err - state[1]) / dt;
        state[1] = err;

        // PIDF: kP·e + kI·∫e + kD·de/dt + kF·targetVelocity
        double vPID = kp * err + ki * state[0] + kd * deriv + kf * targetVel;

        // Add back-EMF compensation so vPID represents error correction around the
        // actual motor operating point rather than around zero
        double vTarget = vPID + vBackEmf;

        // Torque limiting — clamp voltage to what produces maxCurrent
        double iTarget = maxCurrent;
        double vMin = vBackEmf - iTarget * rMotor;
        double vMax = vBackEmf + iTarget * rMotor;
        double vClamped = Math.max(vMin, Math.min(vMax, vTarget));
        double power = vClamped / batt;

        motor.set(power);

        // Update caller's integral/prevErr state
        if (motor == motorL) {
            integralL = state[0]; prevErrL = state[1];
        } else {
            integralR = state[0]; prevErrR = state[1];
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
