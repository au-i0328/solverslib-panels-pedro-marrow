package org.firstinspires.ftc.teamcode.commands;

import com.seattlesolvers.solverslib.command.Command;
import com.skeletonarmyftc.marrow.spatial.zone.PolygonZone;
import com.skeletonarmyftc.marrow.spatial.zone.Point;

import org.firstinspires.ftc.teamcode.RobotHardware;

/**
 * Launch Zone RTP — pull-to-polygon-zone command.
 *
 * - Reads robot pose from odometry + vision
 * - Computes distance to two polygon launch zones (close & far)
 * - Picks the nearer zone
 * - Computes the closest point on that zone's boundary
 * - Blends a pull vector toward that point with the driver's joystick
 * - VECTOR_WEIGHT_DRIVER controls driver override strength
 *
 * Non-blocking — computed every loop.
 */
public class LaunchZoneRTPCommand implements Command {

    private final java.util.function.Supplier<Double> getRobotX;
    private final java.util.function.Supplier<Double> getRobotY;
    private final java.util.function.Supplier<Double> getRobotH; // radians
    private final java.util.function.Supplier<Double> getDriverFwd;  // -1 to 1
    private final java.util.function.Supplier<Double> getDriverStrafe; // -1 to 1
    private final java.util.function.DoubleConsumer setBlendedFwd;
    private final java.util.function.DoubleConsumer setBlendedStrafe;
    private final java.util.function.Supplier<Double> getBlendedFwd; // previous frame's output
    private final java.util.function.Supplier<Double> getBlendedStrafe; // previous frame's output
    private final java.util.function.DoubleConsumer onTargetPointChanged; // optional hook
    /** Supplier for the live robot footprint zone (position/rotation synced each loop). */
    private final java.util.function.Supplier<PolygonZone> getRobotZone;

    private boolean active = false;

    // Live target point — continuously updated while driver is adding input
    private double targetX;
    private double targetY;

    public LaunchZoneRTPCommand(
            java.util.function.Supplier<Double> getRobotX,
            java.util.function.Supplier<Double> getRobotY,
            java.util.function.Supplier<Double> getRobotH,
            java.util.function.Supplier<Double> getDriverFwd,
            java.util.function.Supplier<Double> getDriverStrafe,
            java.util.function.DoubleConsumer setBlendedFwd,
            java.util.function.DoubleConsumer setBlendedStrafe,
            java.util.function.Supplier<Double> getBlendedFwd,
            java.util.function.Supplier<Double> getBlendedStrafe,
            java.util.function.DoubleConsumer onTargetPointChanged,
            java.util.function.Supplier<PolygonZone> getRobotZone) {
        this.getRobotX = getRobotX;
        this.getRobotY = getRobotY;
        this.getRobotH = getRobotH;
        this.getDriverFwd = getDriverFwd;
        this.getDriverStrafe = getDriverStrafe;
        this.setBlendedFwd = setBlendedFwd;
        this.setBlendedStrafe = setBlendedStrafe;
        this.getBlendedFwd = getBlendedFwd;
        this.getBlendedStrafe = getBlendedStrafe;
        this.onTargetPointChanged = onTargetPointChanged;
        this.getRobotZone = getRobotZone;
    }

    /** Call every loop to activate launch zone pull. */
    public void setActive(boolean a) {
        if (a && !this.active) {
            // Fresh activation — sync robot footprint then find closest boundary point
            PolygonZone rz = getRobotZone.get();
            rz.setPosition(getRobotX.get(), getRobotY.get());
            rz.setRotation(getRobotH.get());

            PolygonZone nearest = nearestZone(getRobotX.get(), getRobotY.get());
            double[] cp = closestPointOnPolygon(getRobotX.get(), getRobotY.get(), nearest);
            this.targetX = cp[0];
            this.targetY = cp[1];
        }
        this.active = a;
    }

    @Override
    public void execute() {
        double robotX = getRobotX.get();
        double robotY = getRobotY.get();
        double robotH = getRobotH.get();

        if (active) {
            // Sync robot footprint to live pose
            PolygonZone rz = getRobotZone.get();
            rz.setPosition(robotX, robotY);
            rz.setRotation(robotH);

            // Raw driver input (robot frame)
            double driverFwd    = getDriverFwd.get();
            double driverStrafe = getDriverStrafe.get();
            double rawMag = Math.sqrt(driverFwd * driverFwd + driverStrafe * driverStrafe);

            // Previous frame's blended output (world frame)
            double prevFwd    = getBlendedFwd.get();
            double prevStrafe = getBlendedStrafe.get();
            double prevMag = Math.sqrt(prevFwd * prevFwd + prevStrafe * prevStrafe);

            // If driver is intentionally adding input, update the target to follow
            if (rawMag > 0.1 && rawMag > prevMag * 1.05) {
                PolygonZone nearest = nearestZone(robotX, robotY);
                double[] cp = closestPointOnPolygon(robotX, robotY, nearest);
                if (Double.isFinite(cp[0]) && Double.isFinite(cp[1])) {
                    this.targetX = cp[0];
                    this.targetY = cp[1];
                    if (onTargetPointChanged != null) {
                        onTargetPointChanged.accept(0.0);
                    }
                }
            }

            // Pull unit vector toward the live target point
            double dx = targetX - robotX;
            double dy = targetY - robotY;
            double dist = Math.sqrt(dx * dx + dy * dy);

            double toZoneX, toZoneY;
            if (dist < 0.001) {
                toZoneX = 0.0;
                toZoneY = 0.0;
            } else {
                toZoneX = dx / dist;
                toZoneY = dy / dist;
            }

            // Driver input in world frame (robotH → field orientation)
            double driverWorldX =  driverFwd * Math.cos(robotH) - driverStrafe * Math.sin(robotH);
            double driverWorldY =  driverFwd * Math.sin(robotH) + driverStrafe * Math.cos(robotH);

            // Blend pull vector + driver joystick
            double w = RobotHardware.VECTOR_WEIGHT_DRIVER; // 0.6 default
            double blendedX = toZoneX * (1.0 - w) + driverWorldX * w;
            double blendedY = toZoneY * (1.0 - w) + driverWorldY * w;

            // Clamp to unit magnitude so the drive controller never gets > 1
            double mag = Math.sqrt(blendedX * blendedX + blendedY * blendedY);
            if (mag > 1.0) {
                blendedX /= mag;
                blendedY /= mag;
            }

            setBlendedFwd.accept(blendedX);
            setBlendedStrafe.accept(blendedY);
        } else {
            // Not active — pass raw driver input through unchanged
            setBlendedFwd.accept(getDriverFwd.get());
            setBlendedStrafe.accept(getDriverStrafe.get());
        }
    }

    /**
     * Returns whichever zone is closer to (x, y).
     */
    private PolygonZone nearestZone(double x, double y) {
        double dClose = RobotHardware.CLOSE_LAUNCH_ZONE.distanceTo(x, y);
        double dFar   = RobotHardware.FAR_LAUNCH_ZONE.distanceTo(x, y);
        return (dClose <= dFar) ? RobotHardware.CLOSE_LAUNCH_ZONE : RobotHardware.FAR_LAUNCH_ZONE;
    }

    /**
     * Returns {closestX, closestY} on the polygon boundary nearest to (px, py).
     */
    private double[] closestPointOnPolygon(double px, double py, PolygonZone zone) {
        Point[] verts = zone.getVertices();
        int n = verts.length;

        double bestDist = Double.MAX_VALUE;
        double bestX = px, bestY = py;

        for (int i = 0; i < n; i++) {
            Point a = verts[i];
            Point b = verts[(i + 1) % n]; // next vertex (wraps around)

            double[] cp = closestPointOnSegment(px, py, a.x, a.y, b.x, b.y);
            double d = hypot(px - cp[0], py - cp[1]);

            if (d < bestDist) {
                bestDist = d;
                bestX = cp[0];
                bestY = cp[1];
            }
        }
        return new double[]{ bestX, bestY };
    }

    /**
     * Returns {cx, cy} — the point on segment AB nearest to P.
     * A = (ax, ay), B = (bx, by), P = (px, py).
     */
    private double[] closestPointOnSegment(
            double px, double py,
            double ax, double ay,
            double bx, double by) {

        double bxax = bx - ax;
        double byay = by - ay;
        double segLenSq = bxax * bxax + byay * byay;

        if (segLenSq < 1e-12) {
            // Degenerate — A and B are the same point
            return new double[]{ ax, ay };
        }

        // Projection parameter t ∈ [0, 1]
        double t = ((px - ax) * bxax + (py - ay) * byay) / segLenSq;
        t = Math.max(0.0, Math.min(1.0, t));

        return new double[]{ ax + t * bxax, ay + t * byay };
    }

    private double hypot(double dx, double dy) {
        return Math.sqrt(dx * dx + dy * dy);
    }

    @Override
    public boolean isFinished() {
        return false; // Runs every loop
    }

    @Override
    public void end() {
        setBlendedFwd.accept(getDriverFwd.get());
        setBlendedStrafe.accept(getDriverStrafe.get());
    }
}
