package org.firstinspires.ftc.teamcode.commands;

import com.pedropathing.follower.Follower;
import com.seattlesolvers.solverslib.command.Command;

import org.firstinspires.ftc.teamcode.RobotHardware;

/**
 * Run-to-point (RTP) command using Pedro Pathing Follower.
 * Non-blocking — drives the robot toward a target Pose without blocking the loop.
 *
 * @param follower       Pedro Pathing Follower instance
 * @param x             target X coordinate (Pedro field coords)
 * @param y             target Y coordinate (Pedro field coords)
 * @param heading       target heading in radians
 * @param holdEnd       if true, holds position when path ends; if false, cancels immediately
 * @param maxSpeed      global max speed multiplier (0.0 – 1.0)
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
            follower.follow(
                new com.pedropathing.geometry.Pose(x, y, heading),
                holdEnd
            );
            follower.setMaxPower(maxSpeed);
        }
    }

    @Override
    public boolean isFinished() {
        return !follower.isBusy();
    }

    @Override
    public void end() {
        if (holdEnd) {
            follower.cancelFollow();
        }
    }
}
