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
import com.skeletonarmyftc.marrow.util.Settings;

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
 *   Option → re-zero field-centric drive yaw to current heading (no IMU reset)
 *   Touchpad → set pose to (72, 144) — alliance heading
 *   Share → re-seed pose from Limelight MegaTag (edge-triggered)
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

    // Pose persistence — save every 5 seconds via Marrow Settings
    private final ElapsedTime poseSaveTimer = new ElapsedTime();
    private static final double POSE_SAVE_INTERVAL = 5.0; // seconds
    private static final String SETTING_POSE_X   = "pose_x";
    private static final String SETTING_POSE_Y   = "pose_y";
    private static final String SETTING_POSE_H   = "pose_h";

    // Drive re-zero: captures the IMU yaw at the moment Options is pressed so field-centric
    // drive can re-align its forward direction without modifying the raw IMU used by localization.
    private double driveYawOffset = 0.0;

    // True for one loop tick after initLocalizer() is called; signals the next valid vision
    // reading should anchor the filters immediately (staleness check bypassed) rather than
    // waiting for a fresh frame and growing Kalman uncertainty in the meantime.
    private boolean localizerJustReset = false;

    // Alliance selection (locked after first press)
    private boolean allianceLocked = false;

    // Launch Zone RTP command
    private LaunchZoneRTPCommand launchZoneRTP;

    // Base Zone RTP command
    private BaseZoneRTPCommand baseZoneRTP;

    // Shoot-hood: remember hood position at start of SHOOT for kH compensation
    private boolean shootHoodLocked = false;
    private double lockedHoodPosition = 0.0;

    // Rumble state machine: 50ms on → 50ms off → 50ms on when isReadyToShoot first becomes true
    private enum RumblePhase { OFF, RUMBLE_1, PAUSE, RUMBLE_2 }
    private RumblePhase rumblePhase = RumblePhase.OFF;
    private final com.qualcomm.robotcore.util.ElapsedTime rumbleTimer = new com.qualcomm.robotcore.util.ElapsedTime();

    // Blended drive inputs from launch zone RTP
    private double blendedFwd = 0.0;
    private double blendedStrafe = 0.0;
    private double prevDriverFwd = 0.0;
    private double prevDriverStrafe = 0.0;
    private boolean launchZoneActive = false;

    // Marrow PolygonZone representing the robot's physical footprint (size set in RobotHardware)

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
            strafe -> blendedStrafe = strafe,
            () -> blendedFwd,
            () -> blendedStrafe,
            null,
            () -> RobotHardware.ROBOT_ZONE
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
        // Flag is cleared here; the actual read happens on the first loop() tick (non-blocking).
        poseRestored = false;

        loopTimer.reset();
    }

    @Override
    public void loop() {
        hw.clearBulkCache();
        double dt = loopTimer.seconds();
        loopTimer.reset();

        // ── 1. LOCALIZATION ──────────────────────────────────────
        // Pose restore: runs once on the first loop tick after start().
        // Limelight has had ~1+ loops to warm up by this point, so MegaTag data is available.
        if (!poseRestored) {
            restorePoseFromSettings();
        }

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

        // MegaTag vision update — refine drift estimates when a fresh tag is visible.
        // When the localizer was just reset (pose restore, Touchpad, or Share), accept
        // any valid reading immediately to anchor the filters — don't wait for a fresh
        // frame and let Kalman uncertainty grow in the meantime.
        var result = limelight.getLatestResult();
        boolean useResult = result != null && result.isValid();
        if (useResult) {
            if (!localizerJustReset && result.getStaleness() >= 0.1) {
                useResult = false;
            }
        }
        if (useResult) {
            double[] botpose = result.getBotpose_MT2();
            if (botpose != null && botpose.length >= 6) {
                hw.updateLocalizerFromVision(
                    odoX, odoY, odoH,
                    botpose[0], botpose[1], Math.toRadians(botpose[5])
                );
            }
        }
        if (useResult) {
            localizerJustReset = false;
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

        // Option → re-zero field-centric drive without touching the IMU (edge-triggered)
        if (driver.wasJustPressed(GamepadKeys.Button.OPTIONS)) {
            driveYawOffset = hw.getYawRadians();
        }

        // Touchpad → reset deadwheel odometry pose to match field position (edge-triggered)
        if (driver.wasJustPressed(GamepadKeys.Button.TOUCHPAD)) {
            double targetYaw = (RobotHardware.ALLIANCE == RobotHardware.Alliance.RED) ? 0.0 : Math.PI;
            follower.setPose(new Pose(72, 144, targetYaw));
            hw.imu.resetYaw();
            hw.initLocalizer();
            localizerJustReset = true;
            driveYawOffset = 0.0;
            // Vibrate 200 ms to confirm reset
            driver.rumble(200);
        }

        // Gamepad1 Share → re-seed pose from MegaTag vision (edge-triggered)
        if (driver.wasJustPressed(GamepadKeys.Button.SHARE)) {
            reinitializePoseFromLimelight();
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

        // Right trigger held while aligning/aligned → Launch Zone pull
        // Only activates when robot footprint is NOT even partially in either launch zone
        RobotHardware.ROBOT_ZONE.setPosition(robotX, robotY);
        RobotHardware.ROBOT_ZONE.setRotation(robotH);
        boolean outsideZones = !RobotHardware.ROBOT_ZONE.isInside(RobotHardware.CLOSE_LAUNCH_ZONE)
                            && !RobotHardware.ROBOT_ZONE.isInside(RobotHardware.FAR_LAUNCH_ZONE);
        boolean rightTrigger = driver.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.1;
        launchZoneActive = rightTrigger
                && (state == RobotState.ALIGNING || state == RobotState.ALIGNED)
                && outsideZones;

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
        double yaw = hw.getYawRadians() - driveYawOffset;

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
        // 50ms on → 50ms off → 50ms on: fires once on the rising edge of isReadyToShoot.
        // shareHeld overrides silently without triggering the ready rumble.
        boolean ready = isReadyToShoot(robotX, robotY);
        boolean shareHeld = operator.isDown(GamepadKeys.Button.SHARE);

        if (ready && !wasReady() && !shareHeld) {
            // Rising edge of ready — start rumble sequence
            rumblePhase = RumblePhase.RUMBLE_1;
            rumbleTimer.reset();
            driver.rumble(50);
            operator.rumble(50);
        } else if (rumblePhase != RumblePhase.OFF && rumbleTimer.seconds() >= 1.0) {
            // 50ms on + 50ms off + 50ms on = 150ms total — 1.0s safety timeout
            rumblePhase = RumblePhase.OFF;
            driver.stopRumble();
            operator.stopRumble();
        } else if (rumblePhase == RumblePhase.PAUSE && rumbleTimer.seconds() >= 0.05) {
            // Pause done → second rumble
            rumblePhase = RumblePhase.RUMBLE_2;
            rumbleTimer.reset();
            driver.rumble(50);
            operator.rumble(50);
        } else if (rumblePhase == RumblePhase.RUMBLE_2 && rumbleTimer.seconds() >= 0.05) {
            // Second rumble done
            rumblePhase = RumblePhase.OFF;
            driver.stopRumble();
            operator.stopRumble();
        } else if (!ready && !shareHeld) {
            // Robot is not ready — cancel any in-progress rumble
            rumblePhase = RumblePhase.OFF;
            driver.stopRumble();
            operator.stopRumble();
        }

        // shareHeld override edge detection: 2 blips on rising edge
        if (shareHeld && !wasShareHeld()) {
            driver.rumbleBlips(2);
            operator.rumbleBlips(2);
        }
        setReadyPrev(ready, shareHeld);

        // ── 10. TELEMETRY ──────────────────────────────────
        sendTelemetry(result, robotX, robotY, robotH, ready, shareHeld, state, poseDist,
                baseZoneActive, launchZoneActive);

        // ── 11. POSE PERSISTENCE ─────────────────────────
        // Periodically save current pose to Marrow Settings so it survives across plays
        if (poseSaveTimer.seconds() >= POSE_SAVE_INTERVAL) {
            poseSaveTimer.reset();
            Settings.set(SETTING_POSE_X, robotX);
            Settings.set(SETTING_POSE_Y, robotY);
            Settings.set(SETTING_POSE_H, robotH);
        }
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

    private boolean isReadyToShoot(double robotX, double robotY) {
        if (controller.getState() != RobotState.ALIGNED) return false;
        var r = limelight.getLatestResult();
        if (r == null || !r.isValid()) return false;

        // isReadyToShoot is true when the robot footprint (partial or full) is inside
        // either launch zone, flywheels are up to speed, and pose distance is valid.
        RobotHardware.ROBOT_ZONE.setPosition(robotX, robotY);
        RobotHardware.ROBOT_ZONE.setRotation(robotH);
        boolean inCloseZone = RobotHardware.ROBOT_ZONE.isInside(RobotHardware.CLOSE_LAUNCH_ZONE);
        boolean inFarZone   = RobotHardware.ROBOT_ZONE.isInside(RobotHardware.FAR_LAUNCH_ZONE);
        if (!inCloseZone && !inFarZone) return false;

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
     * Restores the robot pose from Marrow Settings (if saved from a previous play),
     * falling back to MegaTag vision on the first loop tick.
     *
     * Settings are loaded first; if no saved pose exists, MegaTag is used.
     * The IMU yaw is also reset to match the restored heading so Pedro's
     * TwoWheelLocalizer and the drift filter agree.
     *
     * Orientation is set based on alliance:
     *   RED  → 0 radians  (facing toward the red scoring wall)
     *   BLUE → π radians (facing toward the blue scoring wall)
     */
    private void restorePoseFromSettings() {
        if (poseRestored) return;
        poseRestored = true;

        // ── 1. Try Marrow Settings ──────────────────────────────────
        double savedX = Settings.get(SETTING_POSE_X, Double.NaN);
        double savedY = Settings.get(SETTING_POSE_Y, Double.NaN);
        double savedH = Settings.get(SETTING_POSE_H, Double.NaN);

        if (!Double.isNaN(savedX) && !Double.isNaN(savedY) && !Double.isNaN(savedH)) {
            // Valid saved pose found — use it
            double targetYaw = (RobotHardware.ALLIANCE == RobotHardware.Alliance.RED) ? 0.0 : Math.PI;
            follower.setPose(new Pose(savedX, savedY, targetYaw));
            hw.imu.resetYaw();
            hw.initLocalizer();
            localizerJustReset = true;
            driveYawOffset = 0.0;
            return;
        }

        // ── 2. Fallback: seed from MegaTag ─────────────────────────
        reinitializePoseFromLimelight();
    }

    /**
     * Re-initializes the robot pose using the current MegaTag vision estimate.
     * Reads the latest Limelight result, rotates the vision XY into Pedro's frame
     * using the alliance heading, seeds Pedro's localizer, and resets the IMU yaw.
     *
     * Call this to recover from odometry drift at any time during the match.
     */
    private void reinitializePoseFromLimelight() {
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

        // Reset drive re-zero so field-centric forward aligns with the new orientation
        // Anchor the filters on the next valid vision reading without a staleness requirement.
        localizerJustReset = true;
        driveYawOffset = 0.0;
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
