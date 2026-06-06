package org.firstinspires.ftc.teamcode.subsystems;

import static com.seattlesolvers.solverslib.util.MathUtils.clamp;

import com.seattlesolvers.solverslib.hardware.servos.ServoEx;

import org.firstinspires.ftc.teamcode.RobotHardware;

/**
 * Hood subsystem using ServoEx.
 *
 * ServoEx provides power caching (skips redundant writes when delta &lt; cachingTolerance)
 * and cleaner hardware abstraction over the raw SDK Servo. Hardstops are enforced by clamp()
 * on every write since setRange(0, 1) maps the output scale but does not clamp.
 */
public class HoodSubsystem extends com.seattlesolvers.solverslib.command.Subsystem {
    private final ServoEx hoodL;
    private final ServoEx hoodR;

    public HoodSubsystem(RobotHardware hw) {
        this.hoodL = hw.hoodL;
        this.hoodR = hw.hoodR;
    }

    /**
     * Set hood to a raw desired position, clamped to hardstops.
     * Use setForDistance() for normal aiming; this is for manual override.
     */
    public void setRaw(double position) {
        double p = clamp(position, RobotHardware.HOOD_MIN_POSITION, RobotHardware.HOOD_MAX_POSITION);
        hoodL.setPosition(p);
        hoodR.setPosition(p);
    }

    /**
     * Set hood position for a given distance to the goal, with live offset.
     * Applies kH compensation for flywheel velocity drop.
     */
    public void setForDistance(double distance, double offset,
                                double actualVelocity, double targetVelocity) {
        double basePos = RobotHardware.hoodPosition(distance, offset);

        // kH compensation: difference in velocity × coefficient
        double velDrop = targetVelocity - actualVelocity;
        double hoodOffset = velDrop * RobotHardware.HOOD_COMPENSATION_COEFFICIENT;

        double p = clamp(
            basePos + hoodOffset,
            RobotHardware.HOOD_MIN_POSITION,
            RobotHardware.HOOD_MAX_POSITION
        );
        hoodL.setPosition(p);
        hoodR.setPosition(p);
    }

    /**
     * Set both hood servos to the same desired position.
     */
    public void setPosition(double position) {
        double p = clamp(position, RobotHardware.HOOD_MIN_POSITION, RobotHardware.HOOD_MAX_POSITION);
        hoodL.setPosition(p);
        hoodR.setPosition(p);
    }

    public double getPosition() {
        return hoodL.get();
    }
}
