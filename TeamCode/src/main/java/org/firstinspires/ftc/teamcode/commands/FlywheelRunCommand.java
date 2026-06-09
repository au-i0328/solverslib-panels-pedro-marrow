package org.firstinspires.ftc.teamcode.commands;

import java.util.Set;

import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.Subsystem;

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
    private final java.util.function.Supplier<Double> getBatteryVoltage;
    private double limelightDistance; // inches, from Limelight — mutable so it can be updated

    public FlywheelRunCommand(FlywheelSubsystem flywheel, HoodSubsystem hood,
                              double limelightDistance,
                              java.util.function.Supplier<Double> getBatteryVoltage) {
        this.flywheel = flywheel;
        this.hood = hood;
        this.limelightDistance = limelightDistance;
        this.getBatteryVoltage = getBatteryVoltage;
    }

    /**
     * Update the live distance before the next execute() call.
     * Call this every loop after reading fresh Limelight data.
     */
    public void setDistance(double distance) {
        this.limelightDistance = distance;
    }

    @Override
    public void execute() {
        flywheel.update(0.015, getBatteryVoltage.get());

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
    public void end(boolean interrupted) {}

    @Override
    public Set<Subsystem> getRequirements() {
        return Set.of(flywheel, hood);
    }
}
