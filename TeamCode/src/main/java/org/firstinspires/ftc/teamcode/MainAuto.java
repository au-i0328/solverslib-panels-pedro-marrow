package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.paths.BezierLine;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;
import com.seattlesolvers.solverslib.util.TelemetryData;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.FlywheelSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.GateSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.HoodSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

import java.util.List;

/**
 * Autonomous OpMode for DECODE.
 *
 * Uses Pedro Pathing for path following with the Follower.
 * Uses SolversLib command wrappers (FollowPathCommand, TurnToCommand, etc.).
 *
 * Alliance selection via gamepad in init_loop:
 *   Triangle → RED  (pipeline 0, RED_GOAL_COORDS)
 *   Circle  → BLUE (pipeline 1, BLUE_GOAL_COORDS)
 *
 * Flywheels spin up at constant velocity throughout auto.
 * Hood aims from Limelight distance.
 *
 * State machine sequence:
 *   1. Score preload at goal
 *   2. Intake cycles (pickup from field, score at goal) × 3
 *   3. Park
 */
@Autonomous(group = "main")
public class MainAuto extends CommandOpMode {
    private RobotHardware hw;
    private Follower follower;

    private FlywheelSubsystem flywheel;
    private GateSubsystem gate;
    private IntakeSubsystem intake;
    private HoodSubsystem hood;

    private com.qualcomm.hardware.limelightvision.Limelight3A limelight;
    private TelemetryData telemetryData;
    private ElapsedTime loopTimer = new ElapsedTime();
    private boolean allianceSelected = false;

    // Pose targets
    private Pose startPose;
    private Pose scorePose;
    private Pose[] pickupPoses;
    private Pose parkPose;

    // Path chains
    private PathChain scorePreloadChain, scorePickup1Chain, scorePickup2Chain, scorePickup3Chain;
    private PathChain pickup1Chain, pickup2Chain, pickup3Chain, parkChain;

    @Override
    public void initialize() {
        super.reset();

        hw = new RobotHardware();
        hw.init(hardwareMap);

        // Pedro Pathing Follower
        follower = Constants.createFollower(hardwareMap);
        hw.initLocalizer();

        // Subsystems
        flywheel = new FlywheelSubsystem(hw);
        gate = new GateSubsystem(hw);
        intake = new IntakeSubsystem(hw);
        hood = new HoodSubsystem(hw);

        // Panels telemetry
        telemetryData = new TelemetryData(hw.panels.getTelemetry());

        // Limelight
        limelight = hardwareMap.get(com.qualcomm.hardware.limelightvision.Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.start();

        // Bulk caching
        hw.clearBulkCache();

        registerSubsystems(List.of(flywheel, gate, intake, hood));
    }

    private void buildPaths() {
        // Start pose — center of field, facing scoring wall
        startPose = new Pose(72, 72, Math.toRadians(0));

        // Score pose — in front of the goal
        scorePose = new Pose(goalPoseX(), 72, Math.toRadians(0));

        // Pickup positions — adjust these to match actual field layout
        pickupPoses = new Pose[] {
            new Pose(30, 30, Math.toRadians(45)),
            new Pose(50, 30, Math.toRadians(90)),
            new Pose(72, 50, Math.toRadians(135))
        };

        // Park position
        parkPose = new Pose(120, 20, Math.toRadians(180));

        // Score preload
        scorePreloadChain = follower.pathBuilder()
                .addPath(new BezierLine(startPose, scorePose))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
                .build();

        // Intake 1
        pickup1Chain = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickupPoses[0]))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickupPoses[0].getHeading())
                .build();

        scorePickup1Chain = follower.pathBuilder()
                .addPath(new BezierLine(pickupPoses[0], scorePose))
                .setLinearHeadingInterpolation(pickupPoses[0].getHeading(), scorePose.getHeading())
                .build();

        // Intake 2
        pickup2Chain = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickupPoses[1]))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickupPoses[1].getHeading())
                .build();

        scorePickup2Chain = follower.pathBuilder()
                .addPath(new BezierLine(pickupPoses[1], scorePose))
                .setLinearHeadingInterpolation(pickupPoses[1].getHeading(), scorePose.getHeading())
                .build();

        // Intake 3
        pickup3Chain = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickupPoses[2]))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickupPoses[2].getHeading())
                .build();

        scorePickup3Chain = follower.pathBuilder()
                .addPath(new BezierLine(pickupPoses[2], scorePose))
                .setLinearHeadingInterpolation(pickupPoses[2].getHeading(), scorePose.getHeading())
                .build();

        // Park
        parkChain = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, parkPose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), parkPose.getHeading())
                .build();
    }

    private double goalPoseX() {
        // Park slightly in front of the goal for shooting
        return RobotHardware.ALLIANCE == RobotHardware.Alliance.RED ? 138 : 6;
    }

    private void setGoalCoordsForAlliance() {
        if (RobotHardware.ALLIANCE == RobotHardware.Alliance.RED) {
            RobotHardware.GOAL_COORDS = RobotHardware.RED_GOAL_COORDS;
            limelight.pipelineSwitch(RobotHardware.RED_PIPELINE_INDEX);
        } else {
            RobotHardware.GOAL_COORDS = RobotHardware.BLUE_GOAL_COORDS;
            limelight.pipelineSwitch(RobotHardware.BLUE_PIPELINE_INDEX);
        }
    }

    private InstantCommand intakeNote() {
        return new InstantCommand(() -> {
            intake.runForward();
            gate.close();
        });
    }

    private InstantCommand closeGate() {
        return new InstantCommand(() -> gate.close());
    }

    @Override
    public void init_loop() {
        hw.clearBulkCache();

        if (!allianceSelected) {
            if (gamepad1.triangle) {
                RobotHardware.ALLIANCE = RobotHardware.Alliance.RED;
                setGoalCoordsForAlliance();
                allianceSelected = true;
            } else if (gamepad1.circle) {
                RobotHardware.ALLIANCE = RobotHardware.Alliance.BLUE;
                setGoalCoordsForAlliance();
                allianceSelected = true;
            }
        }
        telemetry.update();
    }

    @Override
    public void start() {
        // Build paths after alliance is selected so goal coordinates are correct
        buildPaths();

        // Autonomous command sequence using SolversLib command wrappers
        schedule(
            // Score preload
            new FollowPathCommand(follower, scorePreloadChain),
            new InstantCommand(() -> { gate.open(); }),
            new WaitCommand((long) (RobotHardware.SHOOT_DELAY * 1000)),
            closeGate(),

            // Intake cycle 1
            new FollowPathCommand(follower, pickup1Chain),
            intakeNote(),
            new WaitCommand(500),
            new FollowPathCommand(follower, scorePickup1Chain),
            new InstantCommand(() -> { gate.open(); }),
            new WaitCommand((long) (RobotHardware.SHOOT_DELAY * 1000)),
            closeGate(),

            // Intake cycle 2
            new FollowPathCommand(follower, pickup2Chain),
            intakeNote(),
            new WaitCommand(500),
            new FollowPathCommand(follower, scorePickup2Chain),
            new InstantCommand(() -> { gate.open(); }),
            new WaitCommand((long) (RobotHardware.SHOOT_DELAY * 1000)),
            closeGate(),

            // Intake cycle 3
            new FollowPathCommand(follower, pickup3Chain),
            intakeNote(),
            new WaitCommand(500),
            new FollowPathCommand(follower, scorePickup3Chain),
            new InstantCommand(() -> { gate.open(); }),
            new WaitCommand((long) (RobotHardware.SHOOT_DELAY * 1000)),
            closeGate(),

            // Park
            new FollowPathCommand(follower, parkChain, false)
        );

        follower.setStartingPose(startPose);
        loopTimer.reset();
    }

    @Override
    public void loop() {
        super.loop();

        hw.clearBulkCache();
        double dt = loopTimer.seconds();
        loopTimer.reset();

        // updateRobotOrientation must be called BEFORE getLatestResult every loop
        limelight.updateRobotOrientation(hw.getYawRadians());

        follower.update();

        // Flywheel and hood update every loop
        flywheel.update();
        double dist = getLimelightDistance();
        if (dist > 0) {
            double avgVel = (flywheel.getVelocityL() + flywheel.getVelocityR()) / 2.0;
            hood.setForDistance(dist, 0, avgVel, flywheel.getTargetVelocity());
        }

        // Localizer update
        hw.predictLocalizer(dt);
        var result = limelight.getLatestResult();
        if (result != null && result.isValid() && result.getStaleness() < 0.1) {
            double[] botpose = result.getBotpose_MT2();
            if (botpose != null && botpose.length >= 6) {
                double odoX = follower.getPose().getX();
                double odoY = follower.getPose().getY();
                double odoH = follower.getPose().getHeading();
                hw.updateLocalizerFromVision(
                    odoX, odoY, odoH,
                    botpose[0], botpose[1], Math.toRadians(botpose[5])
                );
            }
        }

        // Telemetry
        telemetryData.addData("Alliance", RobotHardware.ALLIANCE);
        telemetryData.addData("FlywheelL", flywheel.getVelocityL());
        telemetryData.addData("FlywheelR", flywheel.getVelocityR());
        telemetryData.addData("X", hw.getCorrectedX(follower.getPose().getX()));
        telemetryData.addData("Y", hw.getCorrectedY(follower.getPose().getY()));
        telemetryData.addData("H", Math.toDegrees(hw.getCorrectedH(follower.getPose().getHeading())));
        telemetryData.update();
        telemetry.update();
    }

    private double getLimelightDistance() {
        var result = limelight.getLatestResult();
        if (result == null || !result.isValid()) return -1;

        double dist = result.getTy();
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

    @Override
    public void stop() {
        if (limelight != null) limelight.stop();
        flywheel.stop();
        intake.stop();
    }
}
