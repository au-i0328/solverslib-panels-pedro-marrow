package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.Subsystem;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.seattlesolvers.solverslib.util.TelemetryData;

import org.firstinspires.ftc.teamcode.commands.ShootCommand;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.FlywheelSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.GateSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.HoodSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

import java.util.List;

/**
 * Main TeleOp OpMode — DECODE shoot-assist loop.
 *
 * Gamepad1:
 *   Joystick 1 → mecanum drive (forward/strafe)
 *   Joystick 2 → rotation
 *   Right Trigger held → ALIGNING → ALIGNED
 *   Left Trigger (ALIGNED + isReadyToShoot) → SHOOT
 *   Right Bumper → INTAKE
 *   Left Bumper held → INTAKE_REVERSE (king)
 *   Triangle/Circle → alliance selection in INIT
 *
 * Gamepad2:
 *   dpad_up/down → adjust flywheel velocity offset
 *   dpad_left/right → adjust hood angle offset
 *   Circle → return to INTAKE
 *   Share+Option → trigger Base Zone RTP
 *
 * Flywheels run at constant velocity throughout the match.
 * Hood aims from Limelight distance.
 * All state transitions via MasterController.
 */
@TeleOp(group = "main")
public class MainTeleOp extends CommandOpMode {
    private RobotHardware hw;
    private Follower follower;

    // Subsystems
    private DriveSubsystem drive;
    private FlywheelSubsystem flywheel;
    private IntakeSubsystem intake;
    private GateSubsystem gate;
    private HoodSubsystem hood;

    // State machine
    private MasterController controller;

    // Limelight (raw SDK — SolversLib has no Limelight wrapper)
    private com.qualcomm.hardware.limelightvision.Limelight3A limelight;

    // Gamepad helpers
    private GamepadEx driver;
    private GamepadEx operator;

    // Telemetry
    private TelemetryData telemetryData;

    // Loop timing
    private ElapsedTime loopTimer = new ElapsedTime();

    // Alliance
    private boolean allianceSelected = false;

    // Edge-detection state
    private boolean imuResetWasPressed = false;
    private boolean touchpadWasPressed = false;

    // Gamepad vibration for isReadyToShoot
    private boolean wasNotReady = true;

    @Override
    public void initialize() {
        super.reset();

        hw = new RobotHardware();
        hw.init(hardwareMap);

        // Pedro Pathing Follower for localization and base-zone RTP
        follower = Constants.createFollower(hardwareMap);
        follower.startTeleopDrive();

        // Init localizer filters
        hw.initLocalizer();

        // Subsystems
        drive    = new DriveSubsystem(hw, follower);
        flywheel = new FlywheelSubsystem(hw);
        intake   = new IntakeSubsystem(hw);
        gate     = new GateSubsystem(hw);
        hood     = new HoodSubsystem(hw);

        // Master controller
        controller = new MasterController(flywheel, gate, intake);
        controller.setReadyCheck(this::isReadyToShoot);
        controller.setShootDoneCallback(() -> {
            driver.resetGamepadRumble();
            operator.resetGamepadRumble();
        });

        // Gamepad helpers
        driver   = new GamepadEx(gamepad1);
        operator = new GamepadEx(gamepad2);

        // Panels telemetry
        telemetryData = new TelemetryData(hw.panels.getTelemetry());

        // Limelight init
        limelight = hardwareMap.get(com.qualcomm.hardware.limelightvision.Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.start();

        // Register subsystems
        registerSubsystems(List.of(drive, flywheel, intake, gate, hood));

        // Bulk caching
        hw.clearBulkCache();
    }

    @Override
    public void init_loop() {
        hw.clearBulkCache();

        if (!allianceSelected) {
            // Triangle → RED, Circle → BLUE
            if (gamepad1.triangle) {
                RobotHardware.ALLIANCE = RobotHardware.Alliance.RED;
                RobotHardware.GOAL_COORDS = RobotHardware.RED_GOAL_COORDS;
                limelight.pipelineSwitch(RobotHardware.RED_PIPELINE_INDEX);
                allianceSelected = true;
            } else if (gamepad1.circle) {
                RobotHardware.ALLIANCE = RobotHardware.Alliance.BLUE;
                RobotHardware.GOAL_COORDS = RobotHardware.BLUE_GOAL_COORDS;
                limelight.pipelineSwitch(RobotHardware.BLUE_PIPELINE_INDEX);
                allianceSelected = true;
            }
        }

        // Telemetry
        telemetry.update();
    }

    @Override
    public void start() {
        controller.setState(RobotState.INTAKE);
        allianceSelected = true;
        loopTimer.reset();
    }

    @Override
    public void loop() {
        hw.clearBulkCache();
        double dt = loopTimer.seconds();
        loopTimer.reset();

        // ── 1. UPDATE LOCALIZATION ────────────────────────────────
        // Must call updateRobotOrientation BEFORE getLatestResult every loop
        limelight.updateRobotOrientation(hw.getYawRadians());

        follower.update();
        double odoX = follower.getPose().getX();
        double odoY = follower.getPose().getY();
        double odoH = follower.getPose().getHeading();

        hw.predictLocalizer(dt);

        // Vision update if fresh MegaTag 2 reading
        var result = limelight.getLatestResult();
        if (result != null && result.isValid() && result.getStaleness() < 0.1) {
            double[] botpose = result.getBotpose_MT2();
            if (botpose != null && botpose.length >= 6) {
                hw.updateLocalizerFromVision(
                    odoX, odoY, odoH,
                    botpose[0], botpose[1], Math.toRadians(botpose[5])
                );
            }
        }

        double robotX = hw.getCorrectedX(odoX);
        double robotY = hw.getCorrectedY(odoY);
        double robotH = hw.getCorrectedH(odoH);

        // ── 2. GAMEPAD INPUT ──────────────────────────────────────
        driver.readInputs();
        operator.readInputs();

        // Gamepad2 dpad offsets
        if (gamepad2.dpad_up) {
            flywheel.addOffset(RobotHardware.FLYWHEEL_VELOCITY_OFFSET_JUMP);
        }
        if (gamepad2.dpad_down) {
            flywheel.addOffset(-RobotHardware.FLYWHEEL_VELOCITY_OFFSET_JUMP);
        }
        if (gamepad2.dpad_left) {
            RobotHardware.hoodAngleOffset += RobotHardware.HOOD_ANGLE_OFFSET_JUMP;
        }
        if (gamepad2.dpad_right) {
            RobotHardware.hoodAngleOffset -= RobotHardware.HOOD_ANGLE_OFFSET_JUMP;
        }

        // Gamepad2 circle → return to INTAKE
        if (gamepad2.circle) {
            controller.setState(RobotState.INTAKE);
            gate.close();
            intake.stop();
        }

        // Share + Option → Base Zone RTP
        if (gamepad2.share && gamepad2.options) {
            runToBaseZone();
        }

        // Option → reset IMU for field-centric (edge-triggered via raw gamepad)
        if (gamepad1.options && !imuResetWasPressed) {
            hw.imu.resetYaw();
            imuResetWasPressed = true;
        }
        if (!gamepad1.options) imuResetWasPressed = false;

        // Touchpad → reset deadwheel odometry pose (edge-triggered via raw gamepad)
        if (gamepad1.touchpad && !touchpadWasPressed) {
            follower.setPose(new com.pedropathing.geometry.Pose(0, 0, 0));
            hw.initLocalizer(); // reset Kalman filters
            touchpadWasPressed = true;
        }
        if (!gamepad1.touchpad) touchpadWasPressed = false;

        // ── 3. STATE MACHINE ─────────────────────────────────────
        controller.update(gamepad1, gamepad2, dt);
        RobotState state = controller.getState();

        // ── 4. ALIGNMENT (ALIGNING / ALIGNED) ────────────────────
        double limelightDist = getLimelightDistance();

        if (state == RobotState.ALIGNING || state == RobotState.ALIGNED) {
            // Servo to limelight crosshairs for heading
            if (result != null && result.isValid()) {
                double tx = result.getTx();
                double rotationCorrection = tx * 0.03; // P on crosshair
                // Auto-align while allowing driver override
                // Driver still has joystick control
            }
        }

        // ── 5. FLYWHEEL & HOOD (always when distance available) ──
        // Flywheels run continuously at constant velocity per instructions.md
        flywheel.update();

        if (limelightDist > 0 && (state == RobotState.INIT
                || state == RobotState.INTAKE
                || state == RobotState.INTAKE_REVERSE
                || state == RobotState.ALIGNING
                || state == RobotState.ALIGNED)) {
            // Hood always aims when distance is available
            double avgVel = (flywheel.getVelocityL() + flywheel.getVelocityR()) / 2.0;
            hood.setForDistance(
                limelightDist,
                RobotHardware.hoodAngleOffset,
                avgVel,
                flywheel.getTargetVelocity()
            );
        }

        // NOTE: ShootCommand is already executed inside controller.update() while wasShooting.
        // Do NOT call shootCommand.execute() here — it would double-increment the timer.

        // ── 7. DRIVE ─────────────────────────────────────────────
        double fwd = -driver.getLeftY();
        double strafe = -driver.getLeftX();
        double rot = -driver.getRightX();
        double yaw = hw.getYawRadians();

        boolean stalling = hw.isDriveStallingAny();
        drive.driveFieldCentric(fwd, strafe, rot, yaw, hw.batteryVoltage(), stalling);

        // ── 8. isReadyToShoot VIBRATION ──────────────────────────
        boolean ready = isReadyToShoot();
        if (ready && wasNotReady) {
            // Two short bursts of 50ms each
            driver.rumbleBlips(2);
            operator.rumbleBlips(2);
        }
        wasNotReady = !ready;

        // ── 9. TELEMETRY ─────────────────────────────────────────
        sendTelemetry(result, robotX, robotY, robotH, ready, state, limelightDist);
    }

    private double getLimelightDistance() {
        var result = limelight.getLatestResult();
        if (result == null || !result.isValid()) return -1;

        double dist = result.getTy();
        // Simple trig distance: height / tan(angle + mount_angle)
        double mountRad = Math.toRadians(RobotHardware.LIMELIGHT_MOUNT_ANGLE);
        double tyRad = Math.toRadians(dist);
        if (Math.abs(Math.tan(mountRad + tyRad)) < 0.001) return -1;

        double distance = RobotHardware.GOAL_HEIGHT / Math.tan(mountRad + tyRad)
                       + RobotHardware.LIMELIGHT_DISTANCE_OFFSET;

        if (distance < RobotHardware.LIMELIGHT_DIST_MIN
         || distance > RobotHardware.LIMELIGHT_DIST_MAX) {
            return -1;
        }
        return distance;
    }

    private boolean isReadyToShoot() {
        if (controller.getState() != RobotState.ALIGNED) return false;

        var result = limelight.getLatestResult();
        if (result == null || !result.isValid()) return false;

        double velL = flywheel.getVelocityL();
        double velR = flywheel.getVelocityR();
        double target = flywheel.getTargetVelocity();
        double tol = RobotHardware.FLYWHEEL_READY_TOLERANCE;

        boolean velocityOK = Math.abs(velL - target) < tol && Math.abs(velR - target) < tol;
        boolean limelightOK = getLimelightDistance() > 0;
        boolean distOK = !Double.isNaN(getLimelightDistance()) && getLimelightDistance() > 0;

        return velocityOK && limelightOK && distOK;
    }

    private void runToBaseZone() {
        // Use Pedro Pathing Follower to drive to base zone while aligning
        com.pedropathing.geometry.Point base = RobotHardware.baseCoordsForAlliance(RobotHardware.ALLIANCE);
        com.pedropathing.geometry.Pose target =
            new com.pedropathing.geometry.Pose(base.x, base.y, 0);

        // Simple run-to-pose using Follower
        follower.follow(target, false);
        while (follower.isBusy() && opModeIsActive()) {
            hw.clearBulkCache();
            follower.update();
            hw.panels.getTelemetry().update();
        }
        follower.cancelFollow();
    }

    private void sendTelemetry(com.qualcomm.hardware.limelightvision.LLResult result,
                               double robotX, double robotY, double robotH,
                               boolean ready, RobotState state, double limelightDist) {
        telemetryData.addData("Alliance", RobotHardware.ALLIANCE);
        telemetryData.addData("State", state);
        telemetryData.addData("FlywheelL vel", flywheel.getVelocityL());
        telemetryData.addData("FlywheelR vel", flywheel.getVelocityR());
        telemetryData.addData("Flywheel target", flywheel.getTargetVelocity());
        telemetryData.addData("Gate pos", gate.getPosition());
        telemetryData.addData("Hood pos", hood.getPosition());
        telemetryData.addData("isReadyToShoot", ready);
        telemetryData.addData("Limelight dist (in)", limelightDist);
        telemetryData.addData("Robot X", robotX);
        telemetryData.addData("Robot Y", robotY);
        telemetryData.addData("Robot H (deg)", Math.toDegrees(robotH));
        telemetryData.addData("Vel offset", RobotHardware.flywheelVelocityOffset);
        telemetryData.addData("Hood offset", RobotHardware.hoodAngleOffset);

        if (result != null && result.isValid()) {
            telemetryData.addData("Limelight tx", result.getTx());
            telemetryData.addData("Limelight ty", result.getTy());
        }

        telemetryData.update();
        telemetry.update();
    }

    @Override
    public void stop() {
        if (limelight != null) limelight.stop();
        drive.stop();
        flywheel.stop();
        intake.stop();
    }
}
