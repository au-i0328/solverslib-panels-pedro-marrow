package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.follower.Follower;
import com.seattlesolvers.solverslib.command.Subsystem;
import com.seattlesolvers.solverslib.drivebase.MecanumDrive;
import com.seattlesolvers.solverslib.drivebase.RobotDrive.MotorType;
import com.seattlesolvers.solverslib.geometry.Vector2d;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.RobotHardware;

/**
 * Drivetrain subsystem using SolversLib MecanumDrive with voltage-based
 * torque control layered on top.
 *
 * Architecture:
 *   Gamepad sticks → driveFieldCentric() override
 *                  → mecanum field-centric kinematics
 *                  → per-wheel voltage computation
 *                  → torque limiting (clamps voltage within V_EMF ± I_MAX·R)
 *                  → battery-scaled power → motor.set()
 *
 * Zero power behavior is BRAKE so the robot stops firmly when sticks center.
 *
 * Note: the parent MecanumDrive holds motor references; this class overrides
 * driveFieldCentric() so the parent motor handles are never written to.
 */
public class DriveSubsystem extends MecanumDrive implements Subsystem {

    // ── Motor physics (GoBilda 435 RPM — 13.7:1 planetary, 384.5 CPR) ──
    private static final double TPR         = 384.5;                           // ticks/revolution
    private static final double I_STALL    = 9.2;                             // amps at stall
    private static final double R_MOTOR    = 12.0 / I_STALL;                  // Ω  ≈ 1.30
    private static final double V_RESISTOR  = 0.25 * R_MOTOR;                  // V  ≈ 0.326
    private static final double OMEGA_NOLOAD = 435.0 * 2.0 * Math.PI / 60.0; // rad/s ≈ 45.55
    private static final double K_EMF = (12.0 - V_RESISTOR) / OMEGA_NOLOAD;   // V/(rad/s) ≈ 0.256

    /** Max current per motor (amps) — used for both voltage clamping and stall detection. */
    private static final double MAX_CURRENT = RobotHardware.DRIVE_MAX_CURRENT;

    /** Stall-check polling interval in loop iterations. 0 = always check. */
    public static int STALL_CHECK_INTERVAL = 10;

    // Motor indices matching MotorType enum order (FL=0, FR=1, BL=2, BR=3)
    private static final int IDX_FL = MotorType.kFrontLeft.value;
    private static final int IDX_FR = MotorType.kFrontRight.value;
    private static final int IDX_BL = MotorType.kBackLeft.value;
    private static final int IDX_BR = MotorType.kBackRight.value;

    // ── State ─────────────────────────────────────────────────
    private int   stallCounter = 0;
    private boolean stalling   = false;
    // Power scale: 1.0 = full power; ramps down to STALL_POWER_FLOOR when stalling.
    private double stallPowerScale = 1.0;
    private static final double STALL_POWER_FLOOR = 0.3;

    private final MotorEx[] allMotors;
    private final RobotHardware hw;

    public DriveSubsystem(RobotHardware hw, Follower follower) {
        super(true, hw.fl, hw.fr, hw.bl, hw.br);
        this.hw = hw;

        // Store handles so voltage control and stall detection can access motors directly.
        // Parent MecanumDrive also holds references; this class fully overrides
        // driveFieldCentric() so there is no dual-writing.
        allMotors = new MotorEx[]{ hw.fl, hw.fr, hw.bl, hw.br };

        // kV = 12 V / maxVelocity → motor.set(1.0) = maxVelocity
        // setFeedforwardCoefficients lives on Motor (parent class), not MotorEx.
        double maxVel = TPR * 435.0 / 60.0;  // ≈ 2769 ticks/sec for GoBilda 435 RPM
        double kV = 12.0 / maxVel;
        double kS = 0.15;  // V — overcomes static friction
        for (MotorEx m : allMotors) {
            m.setFeedforwardCoefficients(kS, kV);
        }
    }

    /**
     * Full override of MecanumDrive field-centric drive.
     * Applies voltage-based torque control on top of the mecanum kinematics.
     *
     * Per-wheel voltage loop:
     *   vTarget   = baseWheelPower × 12 V
     *   vBackEmf  = K_EMF × ω               (back-EMF opposing motion)
     *   iTarget   = |baseWheelPower| < 0.01 → 0  (coast when near zero)
     *               otherwise → MAX_CURRENT
     *   vMin       = vBackEmf − iTarget × R_MOTOR
     *   vMax       = vBackEmf + iTarget × R_MOTOR
     *   vClamped   = clamp(vTarget, vMin, vMax)
     *   power      = vClamped / batteryVoltage
     */
    @Override
    public void driveFieldCentric(double strafe, double forward, double turn, double gyroAngle) {
        // Apply input curve to joystick inputs
        double exp = RobotHardware.DRIVE_INPUT_CURVE_EXP;
        strafe  = RobotHardware.applyInputCurve(strafe,  exp);
        forward = RobotHardware.applyInputCurve(forward, exp);
        turn    = RobotHardware.applyInputCurve(turn,    exp);

        strafe  = clipRange(strafe);
        forward = clipRange(forward);
        turn    = clipRange(turn);

        // Field-centric rotation
        Vector2d input = new Vector2d(strafe, forward);
        input = input.rotateBy(-gyroAngle);

        double theta = input.angle();

        double[] raw = new double[4];
        raw[IDX_FL] = Math.sin(theta + Math.PI / 4);
        raw[IDX_FR] = Math.sin(theta - Math.PI / 4);
        raw[IDX_BL] = Math.sin(theta - Math.PI / 4);
        raw[IDX_BR] = Math.sin(theta + Math.PI / 4);

        normalize(raw, input.magnitude());

        // Turn correction — left motors +, right motors −
        raw[IDX_FL] += turn;
        raw[IDX_FR] -= turn;
        raw[IDX_BL] += turn;
        raw[IDX_BR] -= turn;

        normalize(raw);

        // Right motors are hardware-inverted (fr.setInverted(true), br.setInverted(true))
        // in RobotHardware.init(), so powers are passed directly.
        applyVoltageControl(
            raw[IDX_FL],
            raw[IDX_FR],
            raw[IDX_BL],
            raw[IDX_BR]
        );
    }

    /**
     * Voltage + torque control for four wheel power fractions.
     * Replaces the raw motor.set() call from MecanumDrive.
     */
    private void applyVoltageControl(double flPow, double frPow, double blPow, double brPow) {
        double batt = hw.batteryVoltage();
        double[] fractions = { flPow, frPow, blPow, brPow };

        stallCounter++;
        boolean anyStalling = false;

        for (int i = 0; i < 4; i++) {
            double frac = fractions[i];
            MotorEx m   = allMotors[i];

            double vTarget  = frac * 12.0;
            double omega    = m.getVelocity() / TPR * 2.0 * Math.PI;  // rad/s
            double vBackEmf = K_EMF * omega;

            // Coast when stick is near zero; otherwise apply torque limiting
            double iTarget = Math.abs(frac) < 0.01
                ? 0.0
                : MAX_CURRENT;

            double vMin = vBackEmf - iTarget * R_MOTOR;
            double vMax = vBackEmf + iTarget * R_MOTOR;
            double vClamped = Math.max(vMin, Math.min(vMax, vTarget));
            double power    = vClamped / batt;

            // Override: scale down all wheel powers when stalling to protect motors
            if (stallPowerScale < 1.0) {
                power *= stallPowerScale;
            }

            m.set(power);

            if (m.getCurrent(CurrentUnit.AMPS) > MAX_CURRENT) {
                anyStalling = true;
            }
        }

        // Ramp stall power scale toward target to avoid sudden jumps.
        // Going UP:   immediate reduction when stalling is detected.
        // Going DOWN: recover gradually to give motors time to stabilize.
        if (anyStalling) {
            stallCounter = 0;
            stalling = true;
            stallPowerScale = Math.max(stallPowerScale - 0.15, STALL_POWER_FLOOR);
        } else if (STALL_CHECK_INTERVAL > 0 && stallCounter >= STALL_CHECK_INTERVAL) {
            stalling = false;
            stallCounter = 0;
            // Recover gradually: step 10 % per interval check
            stallPowerScale = Math.min(stallPowerScale + 0.10, 1.0);
        }
    }

    /** True if any drive motor is currently drawing above MAX_CURRENT. */
    public boolean isStalling() {
        return stalling;
    }

    /** Immediately cut all drive motors to zero (BRAKE holds position). */
    @Override
    public void stop() {
        for (MotorEx m : allMotors) m.stopMotor();
        stalling = false;
        stallCounter = 0;
        stallPowerScale = 1.0;
    }
}
