package org.firstinspires.ftc.teamcode.subsystems;

import com.seattlesolvers.solverslib.hardware.Motor;

/**
 * Intake subsystem using SolversLib Motor (no encoder).
 *
 * Simple setPower with battery voltage compensation.
 * The commanded power is scaled by 12 V / batteryVoltage so the intake
 * runs at consistent speed regardless of battery sag.
 */
public class IntakeSubsystem extends com.seattlesolvers.solverslib.command.Subsystem {
    private final Motor motor;

    public IntakeSubsystem(RobotHardware hw) {
        this.motor = hw.intake;
    }

    /**
     * Drive intake forward at full speed.
     */
    public void runForward() {
        motor.set(RobotHardware.INTAKE_POWER / RobotHardware.batteryVoltage());
    }

    /**
     * Drive intake reverse at full speed.
     */
    public void runReverse() {
        motor.set(-RobotHardware.INTAKE_POWER / RobotHardware.batteryVoltage());
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
}
