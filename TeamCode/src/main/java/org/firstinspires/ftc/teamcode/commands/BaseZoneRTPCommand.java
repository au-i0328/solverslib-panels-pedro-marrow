package org.firstinspires.ftc.teamcode.commands;

import java.util.Set;

import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.Subsystem;
import com.skeletonarmy.marrow.zones.PolygonZone;
import com.skeletonarmy.marrow.zones.Point;

import org.firstinspires.ftc.teamcode.RobotHardware;

/**
 * Base Zone RTP — pull-to-base-zone command.
 *
 * Active when gamepad2's left-stick and right-stick buttons are both held.
 * Computes the pull vector toward the center of the active alliance's base zone,
 * then blends it with the driver's joystick input so the driver retains control
 * while being assisted toward the zone.
 *
 * The robot is modelled as an 18×18 in² zone to add a safety margin inside
 * the 20×20 in² base zone boundary. Re-activated by holding gamepad2 left- and
 * right-stick buttons.
 *
 * Non-blocking — computed every loop.
 */
public class BaseZoneRTPCommand implements Command {

    private final java.util.function.Supplier<Double> getRobotX;
    private final java.util.function.Supplier<Double> getRobotY;
    private final java.util.function.Supplier<Double> getRobotH; // radians
    private final java.util.function.Supplier<Double> getDriverFwd;  // -1 to 1
    private final java.util.function.Supplier<Double> getDriverStrafe; // -1 to 1
    private final java.util.function.DoubleConsumer setBlendedFwd;
    private final java.util.function.DoubleConsumer setBlendedStrafe;

    private boolean active = false;

    public BaseZoneRTPCommand(
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

    /** Call every loop to activate/deactivate base zone pull. */
    public void setActive(boolean a) { this.active = a; }

    @Override
    public void execute() {
        double robotX = getRobotX.get();
        double robotY = getRobotY.get();
        double robotH = getRobotH.get();

        if (active) {
            // Select the alliance-appropriate base zone
            PolygonZone baseZone = RobotHardware.ALLIANCE == RobotHardware.Alliance.RED
                    ? RobotHardware.RED_BASE_ZONE
                    : RobotHardware.BLUE_BASE_ZONE;

            // Compute base zone center from corners (PolygonZone doesn't expose getCenter)
            Point[] corners = baseZone.getCorners();
            double cx = 0, cy = 0;
            for (Point v : corners) { cx += v.getX(); cy += v.getY(); }
            cx /= corners.length;
            cy /= corners.length;

            // Pull unit vector toward the base zone center
            double dx = cx - robotX;
            double dy = cy - robotY;
            double mag = Math.sqrt(dx * dx + dy * dy);

            double toZoneX, toZoneY;
            if (mag < 0.001) {
                toZoneX = 0.0;
                toZoneY = 0.0;
            } else {
                toZoneX = dx / mag;
                toZoneY = dy / mag;
            }

            // Driver input in world frame (field-centric)
            double driverFwd     = getDriverFwd.get();
            double driverStrafe  = getDriverStrafe.get();
            double driverWorldX =  driverFwd * Math.cos(robotH) - driverStrafe * Math.sin(robotH);
            double driverWorldY =  driverFwd * Math.sin(robotH) + driverStrafe * Math.cos(robotH);

            // Blend pull vector + driver joystick; VECTOR_WEIGHT_DRIVER controls driver override
            double w = RobotHardware.VECTOR_WEIGHT_DRIVER;
            double blendedX = toZoneX * (1.0 - w) + driverWorldX * w;
            double blendedY = toZoneY * (1.0 - w) + driverWorldY * w;

            // Clamp to unit magnitude so drive controller never receives > 1
            double totalMag = Math.sqrt(blendedX * blendedX + blendedY * blendedY);
            if (totalMag > 1.0) {
                blendedX /= totalMag;
                blendedY /= totalMag;
            }

            setBlendedFwd.accept(blendedX);
            setBlendedStrafe.accept(blendedY);
        } else {
            // Not active — pass raw driver input through unchanged
            setBlendedFwd.accept(getDriverFwd.get());
            setBlendedStrafe.accept(getDriverStrafe.get());
        }
    }

    @Override
    public boolean isFinished() {
        return false; // Runs every loop
    }

    @Override
    public void end(boolean interrupted) {
        setBlendedFwd.accept(getDriverFwd.get());
        setBlendedStrafe.accept(getDriverStrafe.get());
    }

    @Override
    public Set<Subsystem> getRequirements() {
        return Set.of(); // No subsystem requirements — pure math/vector computation
    }
}
