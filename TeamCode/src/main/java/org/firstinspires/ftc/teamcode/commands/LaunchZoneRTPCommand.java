package org.firstinspires.ftc.teamcode.commands;

import com.seattlesolvers.solverslib.command.Command;

import org.firstinspires.ftc.teamcode.RobotHardware;

/**
 * Launch Zone RTP command per instructions.md:
 *
 * - Get robot pose from odometry + vision
 * - If right trigger held and robot is outside the launch zone,
 *   drive toward the closest calculated launch zone location
 * - Draw a line 5 cm (≈2 inches) inboard from the scoring wall
 * - Vector addition: launch-zone velocity vector + driver's joystick vector,
 *   with VECTOR_WEIGHT_DRIVER controlling driver override strength
 *
 * Non-blocking — computed every loop so the drive loop can apply the blended result.
 */
public class LaunchZoneRTPCommand implements Command {
    /** 5 cm ≈ 2 inches */
    private static final double LAUNCH_ZONE_OFFSET_IN = 2.0;

    private final java.util.function.Supplier<Double> getRobotX;
    private final java.util.function.Supplier<Double> getRobotY;
    private final java.util.function.Supplier<Double> getRobotH; // radians
    private final java.util.function.Supplier<Double> getDriverFwd;  // -1 to 1
    private final java.util.function.Supplier<Double> getDriverStrafe; // -1 to 1
    private final java.util.function.DoubleConsumer setBlendedFwd;
    private final java.util.function.DoubleConsumer setBlendedStrafe;

    private boolean active = false;

    public LaunchZoneRTPCommand(
            java.util.function.Supplier<Double> getRobotX,
            java.util.function.Supplier<Double> getRobotY,
            java.util.function.Supplier<Double> getRobotH,
            java.util.function.Supplier<Double> getDriverFwd,
            java.util.function.Supplier<Double> getDriverStrafe,
            java.util.function.DoubleConsumer setBlendedFwd,
            java.util.function.DoubleConsumer setBlendedStrafe) {
        this.getRobotX = getRobotX;
        this.getRobotY = getRobotY;
        this.getRobotH = getRobotH;
        this.getDriverFwd = getDriverFwd;
        this.getDriverStrafe = getDriverStrafe;
        this.setBlendedFwd = setBlendedFwd;
        this.setBlendedStrafe = setBlendedStrafe;
    }

    /** Call every loop to activate launch zone pull. */
    public void setActive(boolean a) { this.active = a; }

    @Override
    public void execute() {
        double robotX = getRobotX.get();
        double robotY = getRobotY.get();
        double robotH = getRobotH.get();

        // Scoring wall X coordinate for this alliance
        double goalX = RobotHardware.ALLIANCE == RobotHardware.Alliance.RED
                ? RobotHardware.RED_GOAL_COORDS.x
                : RobotHardware.BLUE_GOAL_COORDS.x;

        // Launch zone edge: 5 cm inboard from scoring wall
        double launchX = goalX - LAUNCH_ZONE_OFFSET_IN;

        // Distance from launch zone line (positive = outside, negative = inside)
        double distFromZone = robotX - launchX;

        if (active && distFromZone > 0) {
            // Robot is outside launch zone — compute pull vector toward it
            // Nearest point on launch zone line to robot's current Y
            double targetX = launchX;
            double targetY = robotY;

            // Unit vector from robot to launch zone point
            double dx = targetX - robotX;
            double dy = targetY - robotY;
            double dist = Math.sqrt(dx * dx + dy * dy);
            double toZoneX = (dist < 0.001) ? 0 : dx / dist;
            double toZoneY = (dist < 0.001) ? 0 : dy / dist;

            // Driver input in world frame
            double driverFwd = getDriverFwd.get();
            double driverStrafe = getDriverStrafe.get();
            double driverWorldX = driverFwd * Math.cos(robotH) - driverStrafe * Math.sin(robotH);
            double driverWorldY = driverFwd * Math.sin(robotH) + driverStrafe * Math.cos(robotH);

            // Blend: launch zone pull + driver joystick
            // VECTOR_WEIGHT_DRIVER = 0.6 means driver can override 60% of the pull
            double w = RobotHardware.VECTOR_WEIGHT_DRIVER;
            double blendedX = toZoneX * (1.0 - w) + driverWorldX * w;
            double blendedY = toZoneY * (1.0 - w) + driverWorldY * w;

            // Clamp magnitude to [-1, 1]
            double mag = Math.sqrt(blendedX * blendedX + blendedY * blendedY);
            if (mag > 1.0) {
                blendedX /= mag;
                blendedY /= mag;
            }

            setBlendedFwd.accept(blendedX);
            setBlendedStrafe.accept(blendedY);
        } else {
            // Inside zone or not active — pass through raw driver input
            setBlendedFwd.accept(getDriverFwd.get());
            setBlendedStrafe.accept(getDriverStrafe.get());
        }
    }

    @Override
    public boolean isFinished() {
        return false; // Runs every loop
    }

    @Override
    public void end() {
        // Restore raw driver input on end
        setBlendedFwd.accept(getDriverFwd.get());
        setBlendedStrafe.accept(getDriverStrafe.get());
    }
}
