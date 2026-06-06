package org.firstinspires.ftc.teamcode.subsystems;

import com.seattlesolvers.solverslib.hardware.Motor;

/**
 * Intake subsystem using SolversLib Motor (no encoder).
 *
 * Pure feedforward — no velocity feedback. Since encoder velocity is unavailable,
 * back-EMF is estimated from the *commanded* speed rather than measured speed.
 *
 * Voltage loop per cycle:
 *
 *   ω_command  = command × INTAKE_MAX_VELOCITY / INTAKE_TPR × 2π   (rad/s)
 *   vFF        = K_EMF × ω_command                                  (feedforward)
 *   vS         = kS × sign(command)                                 (static friction)
 *
 *   vBackEmf   = vFF + vS                                          (estimated opposing EMF)
 *
 *   iTarget    = |command| < 0.01 → 0 (coast when stopped)
 *                otherwise → INTAKE_MAX_CURRENT                      (4 A — jam protection)
 *   vMin       = vBackEmf − iTarget × R_MOTOR
 *   vMax       = vBackEmf + iTarget × R_MOTOR
 *   vClamped   = clamp(command × 12 V, vMin, vMax)
 *   power      = vClamped / batteryVoltage
 *
 * Battery compensation is always applied. kH/voltage compensation handles
 * speed variation from battery sag — no closed-loop PID is used.
 */
public class IntakeSubsystem extends com.seaversolvers.solverslib.command.Subsystem {
    private final Motor motor;

    public IntakeSubsystem(RobotHardware hw) {
        this.motor = hw.intake;
    }

    /**
     * Drive intake forward at 100 % speed.
     */
    public void runForward() {
        applyVoltageControl(1.0);
    }

    /**
     * Drive intake reverse at 100 % speed.
     */
    public void runReverse() {
        applyVoltageControl(-1.0);
    }

    /**
     * Stop intake immediately.
     */
    public void stop() {
        motor.stopMotor();
    }

    public boolean isRunning() {
        return Math.abs(motor.get()) > 0.01;
    }

    /**
     * Pure feedforward + voltage compensation, no velocity feedback.
     *
     * @param command fraction of max speed, −1.0 to 1.0
     */
    private void applyVoltageControl(double command) {
        double batt = RobotHardware.batteryVoltage();

        // Commanded (nominal) omega from the speed fraction
        double omegaCmd = command
                * RobotHardware.INTAKE_MAX_VELOCITY
                / RobotHardware.INTAKE_TPR
                * 2.0 * Math.PI; // rad/s

        // Pure feedforward: kS (static friction) + kEMF × ω_cmd
        double vFF = RobotHardware.INTAKE_K_EMF * omegaCmd;
        double vS  = 0.15 * Math.signum(command);  // kS — overcomes static friction
        double vBackEmf = vFF + vS;

        // Jam protection: clamp voltage so motor never exceeds max current
        double iTarget;
        if (Math.abs(command) < 0.01) {
            iTarget = 0.0;  // coast — no resistance when stopped
        } else {
            iTarget = RobotHardware.INTAKE_MAX_CURRENT;
        }

        double vMin = vBackEmf - iTarget * RobotHardware.INTAKE_R;
        double vMax = vBackEmf + iTarget * RobotHardware.INTAKE_R;
        double vClamped = Math.max(vMin, Math.min(vMax, command * 12.0));

        double power = vClamped / batt;
        motor.set(power);
    }
}
