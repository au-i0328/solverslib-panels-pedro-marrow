package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.RobotHardware;

public class GateSubsystem extends com.seattlesolvers.solverslib.command.Subsystem {
    private final Servo gate;

    public GateSubsystem(RobotHardware hw) {
        this.gate = hw.gate;
        gate.setPosition(RobotHardware.GATE_CLOSE_POSITION);
    }

    public void open() {
        gate.setPosition(RobotHardware.GATE_OPEN_POSITION);
    }

    public void close() {
        gate.setPosition(RobotHardware.GATE_CLOSE_POSITION);
    }

    public boolean isOpen() {
        return gate.getPosition() > (RobotHardware.GATE_OPEN_POSITION + RobotHardware.GATE_CLOSE_POSITION) / 2.0;
    }

    public double getPosition() {
        return gate.getPosition();
    }
}
