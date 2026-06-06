package org.firstinspires.ftc.teamcode.subsystems;

import static com.seattlesolvers.solverslib.util.MathUtils.clamp;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.RobotHardware;

public class HoodSubsystem extends com.seattlesolvers.solverslib.command.Subsystem {
    private final Servo hoodL;
    private final Servo hoodR;

    public HoodSubsystem(RobotHardware hw) {
        this.hoodL = hw.hoodL;
        this.hoodR = hw.hoodR;
    }

    /**
     * Set hood to a raw servo position (0–1), clamped to hardstops.
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

        double finalPos = clamp(
            basePos + hoodOffset,
            RobotHardware.HOOD_MIN_POSITION,
            RobotHardware.HOOD_MAX_POSITION
        );

        hoodL.setPosition(finalPos);
        hoodR.setPosition(finalPos);
    }

    /**
     * Set both hood servos to the same position.
     */
    public void setPosition(double position) {
        double p = clamp(position, RobotHardware.HOOD_MIN_POSITION, RobotHardware.HOOD_MAX_POSITION);
        hoodL.setPosition(p);
        hoodR.setPosition(p);
    }

    public double getPosition() {
        return hoodL.getPosition();
    }
}
