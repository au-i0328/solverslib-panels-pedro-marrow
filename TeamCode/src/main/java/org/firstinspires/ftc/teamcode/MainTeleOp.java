package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.Subsystem;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.seattlesolvers.solverslib.util.TelemetryData;

import org.firstinspires.ftc.teamcode.commands.BaseZoneRTPCommand;
import org.firstinspires.ftc.teamcode.commands.LaunchZoneRTPCommand;
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
 *   Left Trigger (ALIGNED + isReadyToShoot) → SHOOT  [edge-triggered]
 *   Right Bumper → INTAKE
 *   Left Bumper held → INTAKE_REVERSE (king)
 *   Triangle/Circle → alliance selection in INIT  [locked after first press]
 *   Option → reset IMU (edge-triggered)
 *   Touchpad → reset deadwheel odometry pose to 0,0,0
 *
 * Gamepad2:
 *   dpad_up/down → adjust flywheel velocity offset
 *   dpad_left/right → adjust hood angle offset
 *   circle → return to INTAKE (disregards everything)
 *   share held → override isReadyToShoot = true while held
 *   gamepad2 left-stick + right-stick buttons held → Base Zone RTP
 *
 * Flywheels run at constant velocity throughout the match.
 * Hood aims from Limelight distance; kH compensation applies during SHOOT.
 * Launch Zone RTP pulls toward launch line with driver vector override.
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

    // Limelight
    private com.qualcomm.hardware.limelightvision.Limelight3A limelight;

    // Gamepad helpers
    private GamepadEx driver;
    private GamepadEx operator;

    // Telemetry
    private TelemetryData telemetryData;

    // Loop timing
    private ElapsedTime loopTimer = new ElapsedTime();

    // One-shot flag: prevents restorePoseFromSettings from running more than once per start
    private boolean poseRestored = false;

    // Alliance selection (locked after first press)
    private boolean allianceLocked = false;

    // Launch Zone RTP command
    private LaunchZoneRTPCommand launchZoneRTP;

    // Base Zone RTP command
    private BaseZoneRTPCommand baseZoneRTP;

    // Shoot-hood: remember hood position at start of SHOOT for kH compensation
    private boolean shootHoodLocked = false;
    private double lockedHoodPosition = 0.0;

    // Blended drive inputs from launch zone RTP
    private double blendedFwd = 0.0;
    private double blendedStrafe = 0.0;
    private boolean launchZoneActive = false;

    @Override
    public void initialize() {
        super.reset();

        hw = new RobotHardware();
        hw.init(hardwareMap);

        // Rebuild constants so every OpMode restart picks up the latest Panels-tuned values
        Constants.rebuild();
        follower = Constants.createFollower(hardwareMap);
        // Do NOT call follower.startTeleopDrive() — Pedro provides pose estimation only.
        // DriveSubsystem handles all TeleOp motor writes via voltage-based control.

        // Init localizer
        hw.initLocalizer();

        // Subsystems
        drive    = new DriveSubsystem(hw, null);
        flywheel = new FlywheelSubsystem(hw);
        intake   = new IntakeSubsystem(hw);
        gate     = new GateSubsystem(hw);
        hood     = new HoodSubsystem(hw);

        // Master controller
        controller = new MasterController(flywheel, gate, intake);
        controller.setLimelightProvider(new MasterController.LimelightProvider() {
            @Override
            public com.qualcomm.hardware.limelightvision.LLResult get() {
                return limelight.getLatestResult();
            }
            @Override
            public double getAngleToGoal() {
                var r = limelight.getLatestResult();
                return r != null ? r.getTx() : 0;
            }
            @Override
            public boolean hasTarget() {
                var r = limelight.getLatestResult();
                return r != null && r.isValid();
            }
        });
        controller.setPoseProvider(new MasterController.PoseProvider() {
            @Override public double getX() {
                return hw.getCorrectedX(follower.getPose().getX());
            }
            @Override public double getY() {
                return hw.getCorrectedY(follower.getPose().getY());
            }
            @Override public double getH() {
                return hw.getCorrectedH(follower.getPose().getHeading());
            }
            @Override public com.pedropathing.geometry.Point getGoalCoords() {
                return RobotHardware.GOAL_COORDS;
            }
        });
        controller.setReadyCheck(this::isReadyToShoot);
        controller.setShootDoneCallback(() -> {
            driver.resetGamepadRumble();
            operator.resetGamepadRumble();
            shootHoodLocked = false;
        });

        // Launch Zone RTP
        launchZoneRTP = new LaunchZoneRTPCommand(
            this::getRobotX,
            this::getRobotY,
            this::getRobotH,
            () -> -driver.getLeftY(),
            () -> -driver.getLeftX(),
            fwd -> blendedFwd = fwd,
            strafe -> blendedStrafe = strafe
        );

        // Base Zone RTP
        baseZoneRTP = new BaseZoneRTPCommand(
            this::getRobotX,
            this::getRobotY,
            this::getRobotH,
            () -> -driver.getLeftY(),
            () -> -driver.getLeftX(),
            fwd -> blendedFwd = fwd,
            strafe -> blendedStrafe = strafe
        );

        // Gamepad helpers
        driver   = new GamepadEx(gamepad1);
        operator = new GamepadEx(gamepad2);

        // Panels telemetry
        telemetryData = new TelemetryData(hw.panels.getTelemetry());

        // Limelight init
        limelight = hardwareMap.get(
            com.qualcomm.hardware.limelightvision.Limelight3A.class, "limelight");
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

        // Alliance selection — edge-triggered via GamepadEx readers
        if (!allianceLocked) {
            GamepadEx driverInit = new GamepadEx(gamepad1);
            driverInit.readButtons();

            boolean triangleNow = driverInit.wasJustPressed(GamepadKeys.Button.TRIANGLE);
            boolean circleNow   = driverInit.wasJustPressed(GamepadKeys.Button.CIRCLE);

            if (triangleNow) {
                RobotHardware.ALLIANCE = RobotHardware.Alliance.RED;
                RobotHardware.GOAL_COORDS = RobotHardware.RED_GOAL_COORDS;
                limelight.pipelineSwitch(RobotHardware.RED_PIPELINE_INDEX);
                allianceLocked = true;
            } else if (circleNow) {
                RobotHardware.ALLIANCE = RobotHardware.Alliance.BLUE;
                RobotHardware.GOAL_COORDS = RobotHardware.BLUE_GOAL_COORDS;
                limelight.pipelineSwitch(RobotHardware.BLUE_PIPELINE_INDEX);
                allianceLocked = true;
            }
        }

        telemetry.update();
    }

    @Override
    public void start() {
        // Do not allow changing alliance
        allianceLocked = true;

        // Reset all state
        controller.reset();
        controller.setState(RobotState.INTAKE);

        // Restore pose from Marrow Settings if available (pose persistence across plays)
        // poseRestored is a one-shot flag — read runs on first loop() call after start()
        poseRestored = false;
        restorePoseFromSettings();

        loopTimer.reset();
    }

    @Override
    public void loop() {
        hw.clearBulkCache();
        double dt = loopTimer.seconds();
        loopTimer.reset();

        // ── 1. LOCALIZATION ──────────────────────────────────────
        // updateRobotOrientation must come BEFORE follower.update() so Pedro's
        // TwoWheelLocalizer and the drift filter share the same yaw snapshot.
        limelight.updateRobotOrientation(hw.getYawRadians());
        follower.update();  // Pedro runs its own vision update inside here

        // Read raw odometry pose — Pedro has already applied its own MegaTag correction
        // internally, so this is better-than-dead-reckoning raw input for the drift filter.
        double odoX = follower.getPose().getX();
        double odoY = follower.getPose().getY();
        double odoH = follower.getPose().getHeading();

        hw.predictLocalizer(dt);  // grow Kalman uncertainty with time

        // MegaTag vision update — refine drift estimates when a fresh tag is visible
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

        // Apply drift corrections on top of Pedro's pose
        double robotX = hw.getCorrectedX(odoX);
        double robotY = hw.getCorrectedY(odoY);
        double robotH = hw.getCorrectedH(odoH);

        // ── 2. GAMEPAD INPUT ──────────────────────────────────
        driver.readButtons();
        operator.readButtons();

        // Gamepad2 dpad offsets — edge-triggered (only on press, not hold)
        if (operator.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
            flywheel.addOffset(RobotHardware.FLYWHEEL_VELOCITY_OFFSET_JUMP);
        }
        if (operator.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
            flywheel.addOffset(-RobotHardware.FLYWHEEL_VELOCITY_OFFSET_JUMP);
        }
        if (operator.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) {
            RobotHardware.hoodAngleOffset += RobotHardware.HOOD_ANGLE_OFFSET_JUMP;
        }
        if (operator.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) {
            RobotHardware.hoodAngleOffset -= RobotHardware.HOOD_ANGLE_OFFSET_JUMP;
        }

        // Option → reset IMU (edge-triggered)
        if (driver.wasJustPressed(GamepadKeys.Button.OPTIONS)) {
            hw.imu.resetYaw();
        }

        // Touchpad → reset deadwheel odometry pose (edge-triggered)
        if (driver.wasJustPressed(GamepadKeys.Button.TOUCHPAD)) {
            follower.setPose(new Pose(0, 0, 0));
            hw.initLocalizer();
        }

        // ── 3. STATE MACHINE ──────────────────────────────────
        controller.update(gamepad1, gamepad2, dt);
        RobotState state = controller.getState();

        // ── 4. ALIGNMENT + FALLBACK HEADING ───────────────────
        double poseDist = getPoseDistance();

        // Auto-align: compute rotation correction from limelight tx
        double rotationCorrection = 0.0;
        if (result != null && result.isValid()) {
            rotationCorrection = result.getTx() * 0.05; // P on crosshair
        } else if (state == RobotState.ALIGNING || state == RobotState.ALIGNED) {
            // Fallback: use odometry to turn toward general direction of goal
            double odomHeadingToGoal = Math.toDegrees(
                Math.atan2(
                    RobotHardware.GOAL_COORDS.y - robotY,
                    RobotHardware.GOAL_COORDS.x - robotX
                )
            );
            double headingError = odomHeadingToGoal - Math.toDegrees(robotH);
            // Normalize to [-180, 180]
            while (headingError > 180)  headingError -= 360;
            while (headingError < -180) headingError += 360;
            rotationCorrection = headingError * 0.05; // P on odometry heading
        }

        // ── 5. BASE ZONE & LAUNCH ZONE RTP ──────────────────────────
        // Gamepad2 left-stick + right-stick buttons held → Base Zone pull (highest priority)
        boolean baseZoneActive = operator.isDown(GamepadKeys.Button.LEFT_STICK_BUTTON)
                             && operator.isDown(GamepadKeys.Button.RIGHT_STICK_BUTTON);

        // Right trigger held while aligning → Launch Zone pull
        boolean rightTrigger = driver.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.5;
        launchZoneActive = rightTrigger && (state == RobotState.ALIGNING || state == RobotState.ALIGNED);

        // Priority: base zone > launch zone > raw driver
        if (baseZoneActive) {
            baseZoneRTP.setActive(true);
            launchZoneRTP.setActive(false);
            baseZoneRTP.execute();
        } else if (launchZoneActive) {
            baseZoneRTP.setActive(false);
            launchZoneRTP.setActive(true);
            launchZoneRTP.execute();
        } else {
            baseZoneRTP.setActive(false);
            launchZoneRTP.setActive(false);
            blendedFwd = -driver.getLeftY();
            blendedStrafe = -driver.getLeftX();
        }

        // ── 6. FLYWHEEL ──────────────────────────────────────
        flywheel.update(dt);

        // ── 7. HOOD ───────────────────────────────────────
        // Instructions.md: hood angle set at all times if distance available
        // During SHOOT: base angle locked but kH compensation still applies
        if (poseDist > 0) {
            double avgVel = (flywheel.getVelocityL() + flywheel.getVelocityR()) / 2.0;

            if (state == RobotState.SHOOT) {
                // Lock base position at moment shoot starts; only apply kH compensation
                if (!shootHoodLocked) {
                    lockedHoodPosition = RobotHardware.hoodPosition(poseDist, RobotHardware.hoodAngleOffset);
                    shootHoodLocked = true;
                }
                double velDrop = flywheel.getTargetVelocity() - avgVel;
                double kHOffset = velDrop * RobotHardware.HOOD_COMPENSATION_COEFFICIENT;
                double finalPos = clamp(
                    lockedHoodPosition + kHOffset,
                    RobotHardware.HOOD_MIN_POSITION,
                    RobotHardware.HOOD_MAX_POSITION
                );
                hood.setPosition(finalPos);
            } else {
                shootHoodLocked = false;
                hood.setForDistance(poseDist, RobotHardware.hoodAngleOffset,
                    avgVel, flywheel.getTargetVelocity());
            }
        }

        // ── 8. DRIVE ─────────────────────────────────────
        double yaw = hw.getYawRadians();

        // During alignment, blend auto-rotation correction with driver rotation
        boolean isAligning = state == RobotState.ALIGNING || state == RobotState.ALIGNED;
        double driverRot = -driver.getRightX();
        double autoRot = rotationCorrection;
        double rot;
        if (isAligning) {
            // Auto-rotation correction + driver override
            rot = driverRot + autoRot;
        } else {
            rot = driverRot;
        }

        boolean stalling = hw.isDriveStallingAny();
        drive.driveFieldCentric(blendedFwd, blendedStrafe, rot, yaw);

        // ── 9. isReadyToShoot VIBRATION ────────────────────
        boolean ready = isReadyToShoot();
        boolean shareHeld = operator.isDown(GamepadKeys.Button.SHARE);
        if ((ready || shareHeld) && (ready != wasReady() || shareHeld != wasShareHeld())) {
            driver.rumbleBlips(2);
            operator.rumbleBlips(2);
        }
        setReadyPrev(ready, shareHeld);

        // ── 10. TELEMETRY ──────────────────────────────────
        sendTelemetry(result, robotX, robotY, robotH, ready, shareHeld, state, poseDist,
                baseZoneActive, launchZoneActive);
    }

    // ─────────────────────────────────────────────────────────────
    // HELPERS
    // ─────────────────────────────────────────────────────────────

    private double getPoseDistance() {
        Point goal = RobotHardware.GOAL_COORDS;
        double dx = goal.x - robotX;
        double dy = goal.y - robotY;
        double dist = Math.sqrt(dx * dx + dy * dy);
        if (dist < RobotHardware.LIMELIGHT_DIST_MIN || dist > RobotHardware.LIMELIGHT_DIST_MAX) return -1;
        return dist;
    }

    private boolean isReadyToShoot() {
        if (controller.getState() != RobotState.ALIGNED) return false;
        var r = limelight.getLatestResult();
        if (r == null || !r.isValid()) return false;

        double velL = flywheel.getVelocityL();
        double velR = flywheel.getVelocityR();
        double target = flywheel.getTargetVelocity();
        double tol = RobotHardware.FLYWHEEL_READY_TOLERANCE;

        boolean velocityOK = Math.abs(velL - target) < tol && Math.abs(velR - target) < tol;
        double dist = getPoseDistance();
        boolean distOK = dist > 0 && !Double.isNaN(dist);

        return velocityOK && distOK;
    }

    private boolean isShareHeld() { return operator.isDown(GamepadKeys.Button.SHARE); }

    // Prev-ready state for edge detection of isReadyToShoot
    private boolean prevReady = false;
    private boolean prevShare = false;
    private boolean wasReady() { return prevReady; }
    private boolean wasShareHeld() { return prevShare; }
    private void setReadyPrev(boolean ready, boolean share) {
        prevReady = ready;
        prevShare = share;
    }

    // ── POSE PERSISTENCE ─────────────────────────────────
    // savePoseToSettings is a no-op placeholder — Marrow Settings integration pending.

    /**
     * Attempts to restore the robot pose from Marrow Settings.
     * If no saved pose is available (first match / fresh deploy), falls back to
     * reading the pose from Limelight MegaTag and seeding Pedro's localizer.
     *
     * Orientation is set based on alliance:
     *   RED  → 0 radians  (facing toward the red scoring wall)
     *   BLUE → π radians (facing toward the blue scoring wall)
     *
     * The IMU is also reset so its yaw aligns with the chosen orientation,
     * ensuring Pedro's TwoWheelLocalizer and the drift filter agree.
     */
    private void restorePoseFromSettings() {
        // TODO: integrate Marrow Settings for pose persistence across plays
        // For now, always seed from MegaTag when a target is visible.

        // One-shot guard — only run once per OpMode start
        if (poseRestored) return;
        poseRestored = true;

        var result = limelight.getLatestResult();
        if (result == null || !result.isValid()) {
            return;
        }

        double[] botpose = result.getBotpose_MT2();
        if (botpose == null || botpose.length < 6) {
            return;
        }

        double visionX = botpose[0];
        double visionY = botpose[1];
        double visionH = Math.toRadians(botpose[5]);

        // Target yaw based on alliance:
        //   RED  → 0 rad  (facing +X / scoring wall)
        //   BLUE → π rad  (facing −X / scoring wall)
        double targetYaw = (RobotHardware.ALLIANCE == RobotHardware.Alliance.RED) ? 0.0 : Math.PI;

        // Rotate MegaTag XY into Pedro's coordinate frame using the yaw offset.
        double yawOffset = targetYaw - visionH;
        double cosOff = Math.cos(yawOffset);
        double sinOff = Math.sin(yawOffset);
        double rotX =  visionX * cosOff + visionY * sinOff;
        double rotY = -visionX * sinOff + visionY * cosOff;

        // Seed Pedro's localizer
        follower.setPose(new Pose(rotX, rotY, targetYaw));

        // Sync IMU yaw to match so TwoWheelLocalizer and drift filter are consistent
        hw.imu.resetYaw();

        // Reset Kalman filters to start clean from the new pose
        hw.initLocalizer();
    }

    // ── POSE GETTERS (for LaunchZoneRTPCommand) ──────────
    private double getRobotX() {
        return hw.getCorrectedX(follower.getPose().getX());
    }
    private double getRobotY() {
        return hw.getCorrectedY(follower.getPose().getY());
    }
    private double getRobotH() {
        return hw.getCorrectedH(follower.getPose().getHeading());
    }

    // ── TELEMETRY ────────────────────────────────────────
    private void sendTelemetry(com.qualcomm.hardware.limelightvision.LLResult result,
                               double robotX, double robotY, double robotH,
                               boolean ready, boolean shareOverride,
                               RobotState state, double poseDist,
                               boolean baseZoneActive, boolean launchZoneActive) {
        telemetryData.addData("Alliance", RobotHardware.ALLIANCE);
        telemetryData.addData("State", state);
        telemetryData.addData("FlywheelL vel", flywheel.getVelocityL());
        telemetryData.addData("FlywheelR vel", flywheel.getVelocityR());
        telemetryData.addData("Flywheel target", flywheel.getTargetVelocity());
        telemetryData.addData("Gate pos", gate.getPosition());
        telemetryData.addData("Hood pos", hood.getPosition());
        telemetryData.addData("isReadyToShoot", ready);
        telemetryData.addData("Share override", shareOverride);
        telemetryData.addData("Pose dist (in)", poseDist);
        telemetryData.addData("Pose X", robotX);
        telemetryData.addData("Pose Y", robotY);
        telemetryData.addData("Robot H (deg)", Math.toDegrees(robotH));
        telemetryData.addData("Vel offset", RobotHardware.flywheelVelocityOffset);
        telemetryData.addData("Hood offset", RobotHardware.hoodAngleOffset);
        telemetryData.addData("Base Zone RTP", baseZoneActive);
        telemetryData.addData("Launch Zone RTP", launchZoneActive);

        if (result != null && result.isValid()) {
            telemetryData.addData("Limelight tx", result.getTx());
            telemetryData.addData("Limelight ty", result.getTy());
        }

        telemetryData.update();
        telemetry.update();
    }

    // ── CLEANUP ─────────────────────────────────────────
    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    @Override
    public void stop() {
        if (limelight != null) limelight.stop();
        drive.stop();
        flywheel.stop();
        intake.stop();
    }
}
