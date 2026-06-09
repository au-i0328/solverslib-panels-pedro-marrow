package org.firstinspires.ftc.teamcode.commands;

import java.util.Set;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.Subsystem;

import org.firstinspires.ftc.teamcode.RobotHardware;

/**
 * Run-to-point (RTP) command using Pedro Pathing Follower.
 * Non-blocking — drives the robot toward a target Pose without blocking the loop.
 *
 * Uses Follower.holdPoint(Pose, boolean) where:
 *   - holdEnd=true → robot holds position when path ends
 *   - holdEnd=false → robot cancels immediately when done
 */
public class RunToPointCommand implements Command {
    private final Follower follower;
    private final double x, y, heading;
    private final boolean holdEnd;
    private final double maxSpeed;
    private boolean started = false;

    public RunToPointCommand(Follower follower, double x, double y, double heading,
                             boolean holdEnd, double maxSpeed) {
        this.follower = follower;
        this.x = x;
        this.y = y;
        this.heading = heading;
        this.holdEnd = holdEnd;
        this.maxSpeed = maxSpeed;
    }

    public RunToPointCommand(Follower follower, double x, double y, double heading) {
        this(follower, x, y, heading, true, 1.0);
    }

    @Override
    public void execute() {
        if (!started) {
            started = true;
            follower.holdPoint(new Pose(x, y, heading), holdEnd);
            follower.setMaxPower(maxSpeed);
        }
    }

    @Override
    public boolean isFinished() {
        return !follower.isBusy();
    }

    @Override
    public void end(boolean interrupted) {
        // Follower auto-holds if holdEnd=true; nothing needed here
    }

    @Override
    public Set<Subsystem> getRequirements() {
        return Set.of(); // Follower does not require any Subsystem
    }
}
