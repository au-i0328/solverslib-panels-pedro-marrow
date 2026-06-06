package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;

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
 *   INIT → INTAKE              : Play pressed
 *   INTAKE / INTAKE_REVERSE / INIT + Left Bumper held → INTAKE_REVERSE
 *   INTAKE / INIT             + Right Trigger held → ALIGNING
 *   ALIGNING + time on target  → ALIGNED
 *   ALIGNED + Left Trigger     + isReadyToShoot → SHOOT
 *   ALIGNING / ALIGNED / SHOOT + Right Trigger released → INTAKE
 *   Any state + Right Bumper pressed → INTAKE
 *   SHOOT (timer done) → INTAKE
 *
 * Left bumper is king — always respected before other inputs.
 * Once SHOOT timer starts, gamepads are neglected until timer expires.
 */
public class MasterController {

    private RobotState state = RobotState.INIT;
    private boolean wasShooting = false;

    private final FlywheelSubsystem flywheel;
    private final GateSubsystem gate;
    private final IntakeSubsystem intake;
    private final ShootCommand shootCommand;

    // Alignment timing
    private double alignmentTimer = 0.0;
    private boolean wasAligning = false;

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

    public MasterController(FlywheelSubsystem flywheel, GateSubsystem gate,
                           IntakeSubsystem intake) {
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

    public RobotState getState() {
        return state;
    }

    public void setState(RobotState s) {
        this.state = s;
    }

    public void update(Gamepad gamepad1, Gamepad gamepad2, double dt) {
        // Once SHOOT timer is running, gamepads are neglected
        if (wasShooting) {
            shootCommand.execute();
            if (shootCommand.isFinished()) {
                shootCommand.end();
                wasShooting = false;
            }
            return;
        }

        boolean leftBumper   = gamepad1.left_bumper || gamepad2.left_bumper;
        boolean rightBumper  = gamepad1.right_bumper || gamepad2.right_bumper;
        boolean rightTrigger = gamepad1.right_trigger > 0.5;
        boolean leftTrigger  = gamepad1.left_trigger > 0.5;
        boolean circle       = gamepad2.circle;

        // ── LEFT BUMPER IS KING ──────────────────────────────────
        if (leftBumper) {
            state = RobotState.INTAKE_REVERSE;
            gate.close();
            intake.runReverse();
            return;
        }

        // Return to intake on right bumper
        if (rightBumper) {
            state = RobotState.INTAKE;
            gate.close();
            intake.stop();
            return;
        }

        switch (state) {
            case INIT:
            case INTAKE:
                intake.runForward();
                gate.close();

                if (rightTrigger) {
                    state = RobotState.ALIGNING;
                    alignmentTimer = 0;
                }
                break;

            case INTAKE_REVERSE:
                // Already handled above — stays until bumper released
                intake.runReverse();
                gate.close();
                if (!leftBumper) {
                    state = RobotState.INTAKE;
                }
                break;

            case ALIGNING:
                if (!rightTrigger) {
                    state = RobotState.INTAKE;
                    break;
                }

                // Accumulate time on target
                if (true) { // TODO: replace with actual alignment-on-target check
                    alignmentTimer += dt;
                } else {
                    alignmentTimer = 0;
                }

                if (alignmentTimer >= RobotHardware.ALIGNMENT_DELAY) {
                    state = RobotState.ALIGNED;
                    alignmentTimer = 0;
                }
                break;

            case ALIGNED:
                if (!rightTrigger) {
                    state = RobotState.INTAKE;
                    break;
                }

                if (leftTrigger && readyCheck != null && readyCheck.isReady()) {
                    state = RobotState.SHOOT;
                    wasShooting = true;
                    shootCommand.execute();
                }
                break;

            case SHOOT:
                // Handled at top of loop — gamepads neglected during shoot
                break;
        }
    }

    public void reset() {
        state = RobotState.INIT;
        wasShooting = false;
        alignmentTimer = 0;
    }
}
