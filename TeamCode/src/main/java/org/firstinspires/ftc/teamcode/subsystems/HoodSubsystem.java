package org.firstinspires.ftc.teamcode.subsystems;

import static com.seattlesolvers.solverslib.util.MathUtils.clamp;

import com.seattlesolvers.solverslib.hardware.servos.ServoExGroup;

import org.firstinspires.ftc.teamcode.RobotHardware;

/**
 * Hood subsystem using ServoExGroup.
 *
 * Phase 2: hoodR is hardware-reversed in RobotHardware, so both servos can be
 * driven with a single position value — the SDK handles the inversion math.
 * Phase 3: all setPosition() / setRaw() / setForDistance() calls write once
 * to the group; caching and direction correction are handled by ServoExGroup.
 *
 * Hardstops are enforced by clamp() on every write since setRange(0, 1) maps
 * the output scale but does not enforce the mechanical limits.
 */
public class HoodSubsystem extends com.seattlesolvers.solverslib.command.Subsystem {
    private final ServoExGroup hood;

    public HoodSubsystem(RobotHardware hw) {
        this.hood = hw.hood;
    }

    /**
     * Send both servos to the same position (hardware-reversed so they move together).
     * Clamped to the [HOOD_MIN_POSITION, HOOD_MAX_POSITION] hardstop range.
     */
    public void setPosition(double position) {
        double p = clamp(position,
                RobotHardware.HOOD_MIN_POSITION,
                RobotHardware.HOOD_MAX_POSITION);
        hood.setPosition(p);
    }

    /**
     * Set hood for a given distance to the goal, with live offset and velocity compensation.
     * @param distance       inches to the goal (from Limelight)
     * @param offset         manual tuning offset (gamepad2 dpad)
     * @param actualVelocity current flywheel velocity (ticks/sec)
     * @param targetVelocity flywheel target velocity (ticks/sec)
     */
    public void setForDistance(double distance, double offset,
                               double actualVelocity, double targetVelocity) {
        double basePos = RobotHardware.hoodPosition(distance, offset);

        // kH compensation: velocity drop × coefficient → additional hood angle
        double velDrop    = targetVelocity - actualVelocity;
        double hoodOffset = velDrop * RobotHardware.HOOD_COMPENSATION_COEFFICIENT;

        setPosition(basePos + hoodOffset);
    }

    /**
     * Raw passthrough — use setPosition() for normal aiming; this bypasses the
     * hardstop clamp for calibration or manual override.
     */
    public void setRaw(double rawPosition) {
        hood.setPosition(rawPosition);
    }

    /** Current position from the group leader (hoodL). */
    public double getPosition() {
        return hood.get();
    }
}
