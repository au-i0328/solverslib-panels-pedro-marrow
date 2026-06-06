Use the solverslib FTC library, Pedropathing, Marrow, and built-in FTC sdk in order to program the following:

init state = init motor, servo, limelight, set gate servo to close
intake state = set gate servo to close, continously run intake power 1 (create command)
intakeReverse state = set gate servo to close, continuosly run intake power -1 (create command)
shoot state = run intake power 1, gate to position open, wait n amount of seconds, go to state Intake
aligning state = robot drivetrain auto align via servoing to limelight crosshairs, if apriltag not visible, use robot pose from odometry to turn drivetrain to the general direction of the goal. time on target needs to be larger than (alignment_delay) before switching to state = ALIGNED
aligned state = limelight crosshair aligns to apriltag

Auto Align Process (only heading, run to pose refer to optional features): Check if tag is detected in limelight, yes -> use limelight servoing to turn to correct heading; no -> use bot pose obtained by dead wheel to turn to the general direction of the goal, and then use limelight crosshair to align.


An input curve needs to be applied any time for the MecanumDrive.

WHile aligning, drivetrain can still be moved but the robot continues to be aligned by continuously adjusting rotation.

when state = intake or intakeReverse or init

when state = ALIGNING

when state = ALIGNED

when state = shoot

on left and right trigger release, return to state = intake

right_bumper pressed -> state = intake
right trigger held -> state = ALIGNING -> ALIGNED
left trigger pressed -> if isReadytoShoot = true, enter state = shoot, timer 2 seconds (adjustable in RobotHardware.java as shootDelay variable) -> state = intake
if isReadytoShoot = false, do nothing and maintain current state.

isReadytoShoot is determined by: state = ALIGNED, Flywheel getVelocity within range, limelight result is valid, limelight has valid target, limelight distance value is reasonable, then true, else false

When isReadytoShoot = true, vibrate gamepad with 2 short bursts of 50ms

Robot Hardware should initialize the following: 
Motor FL, FR, BL, BR (FR and BR Direction Reverse), flywheelL (run with encoder), flywheelR (run with encoder), intake
Odom_pods para perpend
IMU imu (control hub internal imu, pass parameters UP and LEFT)
Servo hoodL hoodR gate
Limelight Camera limelight (public void init() {
    limelight = hardwareMap.get(Limelight3A.class, "limelight");
    limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
    limelight.start(); // This tells Limelight to start looking!
})

When the opmode is initatilized, allow gamepad1 to select alliance by pressing Triangle and Circle, Triangle = RED, Circle = BLUE.

Alliance = RED: limelight set pipeline 0, set goal coords to RED COORDS
Alliance = BLUE: limelight set pipeline 1, set goal coords to BLUE COORDS

RobotHardware.java should allow easy tuning of the following: RED AND BLUE goal coords, RED and BLUE Base Coords, goal heights for limelight, PIDF coefficients of flywheelL and flywheelR, shoot delay (seconds), gate servo open position, close position, Flywheel constant target velocity, readyToShoot tolerance for flywheel velocity, limelight distance range, hood position to distance from goal interpolation table, hood position upper and lwoer hardstop, limelight mounting angle, limelight distance offset, flywheel velocity offset jump, hood angle offset jump, hood compensation coefficient, driveStallCurrentThreshold, aligntment_delay, vectorWeightDriver, odom_initpose

Flywheel velocity for both motors are set to a constant. Run 2 seperate PIDF loops for the two motors. The motors spin for the entirety of the match at the target velocity. 


Master Controller Logic
    direction TB
    [*] --> INIT
    
    state INIT {
        direction LR
        Wait_For_Start --> Alliance_Selection
        Alliance_Selection --> RED: Press Triangle (Pipeline 0)
        Alliance_Selection --> BLUE: Press Circle (Pipeline 1)
    }
    
    INIT --> INTAKE : Play Pressed
    
    INTAKE --> INTAKE_REVERSE : Hold Left Bumper
    INTAKE_REVERSE --> INTAKE : Release Left Bumper
    
    INTAKE --> ALIGNING : Hold Right Trigger
    ALIGNING --> ALIGNED : Limelight Aligns to Tag
    
    ALIGNED --> SHOOT : Left Trigger [IF isReadyToShoot]
    ALIGNED --> ALIGNING : Target Lost / Needs Correction
    
    SHOOT --> INTAKE : 2-Second Timer Complete
    
    %% Trigger Release Logic
    ALIGNING --> INTAKE : Release Right Trigger
    ALIGNED --> INTAKE : Release Right Trigger
    
    %% Bumper Override
    ALIGNING --> INTAKE_REVERSE : Left Bumper Pressed
    ALIGNED --> INTAKE_REVERSE : Left Bumper Pressed
    SHOOT --> INTAKE_REVERSE : Left Bumper Pressed

    Drive Logic
    Joystick 1 --> Drive
    Joystick 2 --> Rotation
    Option --> Vibrate 200ms, Reset IMU for Field-Centric Drive only
    Touchpad --> Reset Deadwheel Odometry Pose to 0,0 and heading to 0
    
    Others
    Share --> set isReadyToShoot to true while pressed, neglecting other factors

 
    Once SHOOT Timer begins, gamepads are neglected. When timer is over, state = INTAKE.

    LEFT_BUMPER is king, it should be respected before any other inputs.

    Gamepad2
    dpad_up: manually add (flywheel velocity offset jump) to flywheel target velocity
    dpad_down: manually subtract (flywheel velocity offset jump) to flywheel target velocity
    dpad_left: manually add (hood angle offset jump) to all hood positions with respect to hardstops
    dpad_right: manually subtract (hood angle offset jump) to all hood positions with respect to hardstops
    circle: instantly returns to state = Intake, disregarding everything else
    share+option held: trigger base zone RTP


Loop Time

Enable Lynx Module Bulk Caching in your init() block.
Set it to LynxModule.BulkCachingMode.MANUAL.
Call clearBulkCache() exactly once at the top of your main while (opModeIsActive()) loop.

Hood
The hood angle is set at all times if a limelight distance reading is available. 

in State = SHOOT, General Hood Angle obtained by the interpolation table is locked, however some compensation for the drop in velocity needs to be applied.

Use a Proportional Compensation Factor ($kH$) for that purpose.
Calculate the difference between the target velocity and the velocity we actually have at that exact millisecond. Multiply that RPM drop by (hood compensation coefficient) to get our hood offset.

When the lookedup angle is exceeds either of the hardstops, make sure the servo is stopped at that hardstop value.

Power Management
When stalling in the drive motors are detected (not any other), reduce power to the individual drive motors until they are not stalling. This overrides anything. Only poll the motor current every 30th loop until it is actually stalling, then poll every loop.

Voltage Control
Implement voltage control (voltage/batt voltage) to all motor controls according to skill.

Base Zone RTP
Get robot pose via odometry and vision. Run to the corresponding colour BASE coords. Triggered by gamepad2 holding share and option together.

Launch Zone RTP
Get robot pose from odom pods and vision, when right trigger is held, check if robot is in launch zone, if not, run to closest calculated launch zone location while aligning via run to pose command. Draw a line with 5cm offset from the launch zone line as our own launch zone. 

Vector Addition: The robot calculates the velocity vector required to drive to the launch zone, and adds the driver's joystick input vectors on top of it. This allows the driver to "dodge" an opponent while the robot still tries to pull itself toward the line.

Write in a very clear command based system so that all commands can be reused.

For ALL deadwheel odometry localization, utilize pedropathing and a setup of 2 deadwheel + Control Hub IMU.

You must not use Thread.sleep() for the shoot delay. You must use a non-blocking ElapsedTime timer to trigger the transition back to INTAKE.

Integrate the use of a FTC Panels to allow dynamic tuning of variables and graphing of telemetry data. 

For telemetry, the following info is necessary: ALliance, Robot State, Both flywheel's velocity, gate position, isReadyToShoot?, Limelight Distance, Robot Pose and Heading, the current value for offsets