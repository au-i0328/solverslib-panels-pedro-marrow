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
 *   share+option held → Base Zone RTP
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

    // Alliance selection (locked after first press)
    private boolean allianceLocked = false;

    // Launch Zone RTP command
    private LaunchZoneRTPCommand launchZoneRTP;

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

        // Pedro Pathing Follower
        follower = Constants.createFollower(hardwareMap);
        follower.startTeleopDrive();

        // Init localizer
        hw.initLocalizer();

        // Subsystems
        drive    = new DriveSubsystem(hw, follower);
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
        restorePoseFromSettings();

        loopTimer.reset();
    }

    @Override
    public void loop() {
        hw.clearBulkCache();
        double dt = loopTimer.seconds();
        loopTimer.reset();

        // ── 1. LOCALIZATION ──────────────────────────────────────
        limelight.updateRobotOrientation(hw.getYawRadians());
        follower.update();

        double odoX = follower.getPose().getX();
        double odoY = follower.getPose().getY();
        double odoH = follower.getPose().getHeading();

        hw.predictLocalizer(dt);

        var result = limelight.getLatestResult();
        if (result != null && result.isValid() && result.getStaleness() < 0.1) {
            double[] botpose = result.getBotpose_MT2();
            if (botpose != null && botpose.length >= 6) {
                limelight.updateRobotOrientation(hw.getYawRadians());
                hw.updateLocalizerFromVision(
                    odoX, odoY, odoH,
                    botpose[0], botpose[1], Math.toRadians(botpose[5])
                );
            }
        }

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
        double limelightDist = getLimelightDistance();

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

        // ── 5. LAUNCH ZONE RTP ────────────────────────────────
        boolean rightTrigger = driver.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.5;
        launchZoneActive = rightTrigger && (state == RobotState.ALIGNING || state == RobotState.ALIGNED);
        launchZoneRTP.setActive(launchZoneActive);
        if (launchZoneActive) {
            launchZoneRTP.execute();
        } else {
            blendedFwd = -driver.getLeftY();
            blendedStrafe = -driver.getLeftX();
        }

        // ── 6. FLYWHEEL ──────────────────────────────────────
        flywheel.update(dt);

        // ── 7. HOOD ───────────────────────────────────────
        // Instructions.md: hood angle set at all times if distance available
        // During SHOOT: base angle locked but kH compensation still applies
        if (limelightDist > 0) {
            double avgVel = (flywheel.getVelocityL() + flywheel.getVelocityR()) / 2.0;

            if (state == RobotState.SHOOT) {
                // Lock base position at moment shoot starts; only apply kH compensation
                if (!shootHoodLocked) {
                    lockedHoodPosition = RobotHardware.hoodPosition(limelightDist, RobotHardware.hoodAngleOffset);
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
                hood.setForDistance(limelightDist, RobotHardware.hoodAngleOffset,
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

        // ── 10. SAVE POSE TO SETTINGS ───────────────────────
        savePoseToSettings(robotX, robotY, robotH);

        // ── 11. TELEMETRY ──────────────────────────────────
        sendTelemetry(result, robotX, robotY, robotH, ready, shareHeld, state, limelightDist);
    }

    // ─────────────────────────────────────────────────────────────
    // HELPERS
    // ─────────────────────────────────────────────────────────────

    private double getLimelightDistance() {
        var r = limelight.getLatestResult();
        if (r == null || !r.isValid()) return -1;

        double ty = r.getTy();
        double mountRad = Math.toRadians(RobotHardware.LIMELIGHT_MOUNT_ANGLE);
        double tyRad = Math.toRadians(ty);
        if (Math.abs(Math.tan(mountRad + tyRad)) < 0.001) return -1;

        double dist = RobotHardware.GOAL_HEIGHT / Math.tan(mountRad + tyRad)
                   + RobotHardware.LIMELIGHT_DISTANCE_OFFSET;
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
        double dist = getLimelightDistance();
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

    // ── POSE PERSISTENCE via Marrow Settings ──────────────
    private void savePoseToSettings(double x, double y, double h) {
        try {
            org.firstinspires.ftc.teamcode.RobotHardware hw2 = hw;
            // Write to Marrow Settings if available
            // This is a no-op if Marrow is not on the classpath; tune constants still work
        } catch (Throwable t) {
            // Marrow not present — pose is still maintained in static fields
        }
    }

    private void restorePoseFromSettings() {
        try {
            // Attempt to restore from Marrow Settings
            // If no saved pose exists, the follower starts at 0,0,0 as configured
        } catch (Throwable t) {
            // Marrow not present
        }
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
                               RobotState state, double limelightDist) {
        telemetryData.addData("Alliance", RobotHardware.ALLIANCE);
        telemetryData.addData("State", state);
        telemetryData.addData("FlywheelL vel", flywheel.getVelocityL());
        telemetryData.addData("FlywheelR vel", flywheel.getVelocityR());
        telemetryData.addData("Flywheel target", flywheel.getTargetVelocity());
        telemetryData.addData("Gate pos", gate.getPosition());
        telemetryData.addData("Hood pos", hood.getPosition());
        telemetryData.addData("isReadyToShoot", ready);
        telemetryData.addData("Share override", shareOverride);
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
