package org.firstinspires.ftc.teamcode;

/**
 * Robot state machine states.
 * All state transitions are driven from the MasterController.
 */
public enum RobotState {
    INIT,
    INTAKE,
    INTAKE_REVERSE,
    ALIGNING,
    ALIGNED,
    SHOOT
}
