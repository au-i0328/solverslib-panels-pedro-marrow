package org.firstinspires.ftc.teamcode.subsystems;

import static com.seattlesolvers.solverslib.util.MathUtils.clamp;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.RobotHardware;

/**
 * Hood subsystem using a pair of FTC SDK Servos.
 *
 * The two hood servos are wired so that both physically rotate to the same angle
 * when given the same position value (hoodR's direction is set to REVERSE in
 * RobotHardware.init()). Both servos are written with the same position each
 * loop so they stay in sync.
 *
 * Hardstops are enforced by clamp() on every write.
 */
public class HoodSubsystem implements com.seattlesolvers.solverslib.command.Subsystem {
    private final Servo[] hood;

    public HoodSubsystem(RobotHardware hw) {
        this.hood = hw.hood;  // [0]=hoodL, [1]=hoodR
    }

    /**
     * Send both servos to the same position (hardware-reversed so they move together).
     * Clamped to the [HOOD_MIN_POSITION, HOOD_MAX_POSITION] hardstop range.
     */
    public void setPosition(double position) {
        double p = clamp(position,
                RobotHardware.HOOD_MIN_POSITION,
                RobotHardware.HOOD_MAX_POSITION);
        hood[0].setPosition(p);
        hood[1].setPosition(p);
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
        hood[0].setPosition(rawPosition);
        hood[1].setPosition(rawPosition);
    }

    /** Current position from the leader servo (hoodL). */
    public double getPosition() {
        return hood[0].getPosition();
    }
}
