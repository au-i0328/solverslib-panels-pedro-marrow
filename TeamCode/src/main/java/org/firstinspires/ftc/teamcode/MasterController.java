package org.firstinspires.ftc.teamcode;

import com.pedropathing.geometry.Point;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.commands.ShootCommand;
import org.firstinspires.ftc.teamcode.subsystems.FlywheelSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.GateSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

/**
 * Central state machine controller for the driver-assist loop.
 * Reads gamepad input and drives the robot state machine.
 *
 * State transitions per instructions.md:
 *
 *   INIT --> INTAKE          : Play pressed
 *
 *   INTAKE / INIT + Left Bumper held --> INTAKE_REVERSE
 *   INTAKE_REVERSE --> INTAKE : Release Left Bumper
 *
 *   INTAKE / INIT + Right Trigger held --> ALIGNING
 *   ALIGNING + time on target --> ALIGNED
 *   ALIGNED --> ALIGNING        : Target lost / needs correction
 *
 *   ALIGNED + Left Trigger [isReadyToShoot] --> SHOOT
 *   SHOOT (timer done) --> INTAKE
 *
 *   ALIGNING / ALIGNED / SHOOT + Right Trigger released --> INTAKE
 *
 *   ALIGNING / ALIGNED / SHOOT + Left Bumper pressed --> INTAKE_REVERSE
 *
 *   gamepad2.share held: override isReadyToShoot = true (neglects other factors)
 *   gamepad2.circle: instantly return to INTAKE (disregards everything)
 *
 * LEFT BUMPER is king — always checked first.
 * Once SHOOT timer begins, gamepads are neglected until timer expires.
 */
public class MasterController {

    private RobotState state = RobotState.INIT;
    private boolean wasShooting = false;

    private final FlywheelSubsystem flywheel;
    private final GateSubsystem gate;
    private final IntakeSubsystem intake;
    private final ShootCommand shootCommand;

    // Alignment timing — timer accumulates only while on target; resets on loss
    private ElapsedTime alignmentTimer = new ElapsedTime();
    private boolean aligningActive = false; // true once crosshair first comes on target
    private boolean wasAligned = false;

    // isReadyToShoot gating
    public interface ReadyCheck {
        boolean isReady();
    }
    private ReadyCheck readyCheck;

    // Shoot done callback
    public interface ShootDone {
        void onShootDone();
    }
    private ShootDone shootDone;

    // Callbacks for limelight data (avoids needing hardware in controller)
    public interface LimelightProvider {
        /** Returns the latest LLResult, or null if none. */
        com.qualcomm.hardware.limelightvision.LLResult get();
        /** Returns the angle to the goal in degrees. */
        double getAngleToGoal();
        /** Returns true if an AprilTag is currently visible and valid. */
        boolean hasTarget();
    }
    private LimelightProvider limelightProvider;

    // Callbacks for robot pose
    public interface PoseProvider {
        double getX();
        double getY();
        double getH(); // radians
        Point getGoalCoords();
    }
    private PoseProvider poseProvider;

    // Callbacks for gamepad state
    private boolean prevLeftTrigger = false;
    private boolean prevShare = false;

    public MasterController(FlywheelSubsystem flywheel, GateSubsystem gate, IntakeSubsystem intake) {
        this.flywheel = flywheel;
        this.gate = gate;
        this.intake = intake;
        this.shootCommand = new ShootCommand(flywheel, gate, intake, () -> {
            state = RobotState.INTAKE;
            if (shootDone != null) shootDone.onShootDone();
        });
    }

    public void setReadyCheck(ReadyCheck check) {
        this.readyCheck = check;
    }

    public void setShootDoneCallback(ShootDone done) {
        this.shootDone = done;
    }

    public void setLimelightProvider(LimelightProvider limelight) {
        this.limelightProvider = limelight;
    }

    public void setPoseProvider(PoseProvider pose) {
        this.poseProvider = pose;
    }

    public RobotState getState() {
        return state;
    }

    public void setState(RobotState s) {
        this.state = s;
    }

    /**
     * Main update — call every loop.
     *
     * @param gamepad1   driver gamepad
     * @param gamepad2   operator gamepad
     * @param dt         loop time in seconds
     */
    public void update(Gamepad gamepad1, Gamepad gamepad2, double dt) {
        // Once SHOOT timer is running, gamepads are neglected per instructions.md
        if (wasShooting) {
            shootCommand.execute();
            if (shootCommand.isFinished()) {
                shootCommand.end();
                wasShooting = false;
            }
            return;
        }

        // ── EDGE DETECTION ─────────────────────────────────────────
        boolean leftBumper   = gamepad1.left_bumper || gamepad2.left_bumper;
        boolean rightBumper  = gamepad1.right_bumper || gamepad2.right_bumper;
        boolean rightTrigger = gamepad1.right_trigger > 0.5;

        // Left trigger: edge-triggered (pressed, not held)
        boolean leftTriggerRising = (gamepad1.left_trigger > 0.5) && !prevLeftTrigger;
        prevLeftTrigger = gamepad1.left_trigger > 0.5;

        // Share: edge-detected for override
        boolean shareRising = gamepad2.share && !prevShare;
        prevShare = gamepad2.share;

        // Circle → instant INTAKE (disregards everything else)
        if (gamepad2.circle) {
            state = RobotState.INTAKE;
            gate.close();
            intake.stop();
            return;
        }

        // ── LEFT BUMPER IS KING ────────────────────────────────────
        if (leftBumper) {
            state = RobotState.INTAKE_REVERSE;
            gate.close();
            intake.runReverse();
            return;
        }

        // Right bumper → INTAKE
        if (rightBumper) {
            state = RobotState.INTAKE;
            gate.close();
            intake.stop();
            return;
        }

        // ── PER-STATE LOGIC ──────────────────────────────────────
        switch (state) {
            case INIT:
            case INTAKE:
                intake.runForward();
                gate.close();

                if (rightTrigger) {
                    state = RobotState.ALIGNING;
                    alignmentTimer.reset();
                    wasAligned = false;
                }
                break;

            case INTAKE_REVERSE:
                // Handled above — bumper was held
                intake.runReverse();
                gate.close();
                break;

            case ALIGNING: {
                if (!rightTrigger) {
                    state = RobotState.INTAKE;
                    alignmentTimer.reset();
                    aligningActive = false;
                    break;
                }

                // Auto Align Process (instructions.md):
                // Tag visible → limelight crosshair servoing
                // No tag → odometry heading toward goal → then limelight crosshair
                if (limelightProvider != null && limelightProvider.hasTarget()) {
                    // Tag visible: accumulate time only while crosshair is on target
                    double tx = Math.abs(limelightProvider.getAngleToGoal());
                    double onTargetThreshold = 2.0; // degrees — tune via RobotHardware
                    if (tx < onTargetThreshold) {
                        // Crosshair aligned: accumulate time
                        aligningActive = true;
                    } else {
                        // Crosshair broken: reset timer and restart alignment
                        alignmentTimer.reset();
                        aligningActive = false;
                    }

                    if (aligningActive && alignmentTimer.seconds() >= RobotHardware.ALIGNMENT_DELAY) {
                        state = RobotState.ALIGNED;
                        alignmentTimer.reset();
                        aligningActive = false;
                    }
                } else {
                    // No tag: keep odometry heading toward goal; do NOT transition to ALIGNED
                    // without a visible tag. Keep timer reset so alignment can restart fresh
                    // when the tag comes back into view.
                    alignmentTimer.reset();
                    aligningActive = false;
                }
                break;
            }

            case ALIGNED: {
                if (!rightTrigger) {
                    state = RobotState.INTAKE;
                    alignmentTimer.reset();
                    wasAligned = false;
                    break;
                }

                // Target lost / needs correction → back to ALIGNING
                if (limelightProvider != null && !limelightProvider.hasTarget()) {
                    state = RobotState.ALIGNING;
                    alignmentTimer.reset();
                    wasAligned = false;
                    break;
                }

                // Left trigger (edge-triggered) fires the shot
                if (leftTriggerRising) {
                    // isReadyToShoot is normally checked here, but gamepad2.share overrides it
                    boolean readyOverride = shareRising && gamepad2.share;
                    if (readyOverride || (readyCheck != null && readyCheck.isReady())) {
                        state = RobotState.SHOOT;
                        wasShooting = true;
                        shootCommand.execute();
                    }
                    // If not ready and not overriding, do nothing — maintain ALIGNED
                }
                break;
            }

            case SHOOT:
                // Handled at top of loop — gamepads neglected during shoot
                break;
        }
    }

    /** Returns the current robot heading to the goal from odometry (radians). */
    public double headingToGoalFromOdom() {
        if (poseProvider == null) return 0;
        double rx = poseProvider.getX();
        double ry = poseProvider.getY();
        Point goal = poseProvider.getGoalCoords();
        return Math.atan2(goal.y - ry, goal.x - rx);
    }

    public void reset() {
        state = RobotState.INIT;
        wasShooting = false;
        prevLeftTrigger = false;
        prevShare = false;
        alignmentTimer.reset();
        aligningActive = false;
    }
}
