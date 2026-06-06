package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.RobotHardware;

public class IntakeSubsystem extends com.seattlesolvers.solverslib.command.Subsystem {
    private final DcMotorEx motor;

    public IntakeSubsystem(RobotHardware hw) {
        this.motor = hw.intake;
    }

    public void runForward() {
        motor.setPower(1.0);
    }

    public void runReverse() {
        motor.setPower(-1.0);
    }

    public void stop() {
        motor.setPower(0);
    }

    public boolean isRunning() {
        return Math.abs(motor.getPower()) > 0.01;
    }
}
