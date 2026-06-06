package org.firstinspires.ftc.teamcode.commands;

import com.seattlesolvers.solverslib.command.Command;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.FlywheelSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.HoodSubsystem;

/**
 * Continuously keeps flywheels spinning at target velocity and hood
 * aimed at the goal using Limelight distance.
 */
public class FlywheelRunCommand implements Command {
    private final FlywheelSubsystem flywheel;
    private final HoodSubsystem hood;
    private final double limelightDistance; // inches, from Limelight

    public FlywheelRunCommand(FlywheelSubsystem flywheel, HoodSubsystem hood,
                              double limelightDistance) {
        this.flywheel = flywheel;
        this.hood = hood;
        this.limelightDistance = limelightDistance;
    }

    @Override
    public void execute() {
        flywheel.update();

        if (limelightDistance > 0) {
            hood.setForDistance(
                limelightDistance,
                RobotHardware.hoodAngleOffset,
                (flywheel.getVelocityL() + flywheel.getVelocityR()) / 2.0,
                flywheel.getTargetVelocity()
            );
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end() {}
}
