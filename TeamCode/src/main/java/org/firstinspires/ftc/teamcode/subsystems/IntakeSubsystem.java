package org.firstinspires.ftc.teamcode.subsystems;

import com.seattlesolvers.solverslib.hardware.Motor;

/**
 * Intake subsystem using SolversLib Motor.
 */
public class IntakeSubsystem extends com.seattlesolvers.solverslib.command.Subsystem {
    private final Motor motor;

    public IntakeSubsystem(RobotHardware hw) {
        this.motor = hw.intake;
    }

    public void runForward() {
        motor.set(1.0);
    }

    public void runReverse() {
        motor.set(-1.0);
    }

    public void stop() {
        motor.stopMotor();
    }

    public boolean isRunning() {
        return Math.abs(motor.get()) > 0.01;
    }
}
