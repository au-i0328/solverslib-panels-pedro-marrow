package org.firstinspires.ftc.teamcode.commands;

import com.seattlesolvers.solverslib.command.Command;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.RobotState;
import org.firstinspires.ftc.teamcode.subsystems.FlywheelSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.GateSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

/**
 * Shoot sequence: open gate, run intake power 1, wait shootDelay seconds via ElapsedTime,
 * then transition to INTAKE.
 * Non-blocking — does NOT use Thread.sleep.
 */
public class ShootCommand implements Command {
    private final FlywheelSubsystem flywheel;
    private final GateSubsystem gate;
    private final IntakeSubsystem intake;
    private final Runnable onComplete;

    private final com.qualcomm.robotcore.util.ElapsedTime timer =
        new com.qualcomm.robotcore.util.ElapsedTime();
    private boolean started = false;

    public ShootCommand(FlywheelSubsystem flywheel, GateSubsystem gate,
                        IntakeSubsystem intake, Runnable onComplete) {
        this.flywheel = flywheel;
        this.gate = gate;
        this.intake = intake;
        this.onComplete = onComplete;
    }

    @Override
    public void execute() {
        if (!started) {
            started = true;
            timer.reset();
            gate.open();
            intake.runForward();
            return;
        }

        // Run intake to push note through
        intake.runForward();

        if (timer.seconds() >= RobotHardware.SHOOT_DELAY) {
            gate.close();
            intake.stop();
            if (onComplete != null) onComplete.run();
        }
    }

    @Override
    public boolean isFinished() {
        return started && timer.seconds() >= RobotHardware.SHOOT_DELAY;
    }

    @Override
    public void end() {
        gate.close();
        intake.stop();
    }
}
