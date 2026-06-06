package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.follower.Follower;
import com.seattlesolvers.solverslib.drivebase.MecanumDrive;
import com.seattlesolvers.solverslib.geometry.Vector2d;
import com.seattlesolvers.solverslib.hardware.MotorEx;
import com.seattlesolvers.solverslib.hardware.Motor;
import com.qualcomm.robotcore.hardware.DcMotorEx.CurrentUnit;

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
 */
public class DriveSubsystem extends MecanumDrive {

    // ── Per-motor handles ──────────────────────────────────────
    private final MotorEx fl, fr, bl, br;

    // ── Motor physics (GoBilda 435 RPM — 13.7:1 planetary, 384.5 CPR) ──
    private static final double TPR         = 384.5;         // ticks/revolution
    private static final double I_STALL    = 9.2;          // amps at stall
    private static final double R_MOTOR    = 12.0 / I_STALL;             // Ω  ≈ 1.30
    private static final double V_RESISTOR  = 0.25 * R_MOTOR;             // V  ≈ 0.326  (0.25 A no-load)
    // No-load omega: 435 RPM @ 12 V
    private static final double OMEGA_NOLOAD = 435.0 * 2.0 * Math.PI / 60.0; // rad/s ≈ 45.55
    // Back-EMF: V = V_resistor + kEMF·ω  →  kEMF = (12 − V_res) / ω_nl
    private static final double K_EMF = (12.0 - V_RESISTOR) / OMEGA_NOLOAD; // V/(rad/s) ≈ 0.256

    // rightSideMultiplier = −1 in MecanumDrive when autoInvert = true
    // We replicate it here to apply correct sign after voltage control
    private static final double RIGHT_SIDE = -1.0;

    /** Max current per motor (amps) — triggers voltage clamping. */
    public static double MAX_CURRENT = 3.0;

    /** Stall-check polling interval in loop iterations. 0 = always check. */
    public static int STALL_CHECK_INTERVAL = 10;

    // ── State ─────────────────────────────────────────────────
    private final MotorEx[] allMotors;
    private int  stallCounter = 0;
    private boolean stalling  = false;

    public DriveSubsystem(RobotHardware hw, Follower follower) {
        super(true, hw.fl, hw.fr, hw.bl, hw.br);

        this.fl = hw.fl;
        this.fr = hw.fr;
        this.bl = hw.bl;
        this.br = hw.br;
        this.allMotors = new MotorEx[]{ fl, fr, bl, br };

        // kV = 12 V / maxVelocity → motor.set(1.0) = maxVelocity
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
        strafe = clipRange(strafe);
        forward = clipRange(forward);
        turn    = clipRange(turn);

        // Field-centric rotation
        Vector2d input = new Vector2d(strafe, forward);
        input = input.rotateBy(-gyroAngle);

        double theta = input.angle();

        double[] raw = new double[4];
        raw[kFrontLeft]  = Math.sin(theta + Math.PI / 4);
        raw[kFrontRight] = Math.sin(theta - Math.PI / 4);
        raw[kBackLeft]   = Math.sin(theta - Math.PI / 4);
        raw[kBackRight]  = Math.sin(theta + Math.PI / 4);

        normalize(raw, input.magnitude());

        // Turn correction — left motors +, right motors −
        raw[kFrontLeft]  += turn;
        raw[kFrontRight] -= turn;
        raw[kBackLeft]   += turn;
        raw[kBackRight]  -= turn;

        normalize(raw);

        // Voltage + torque control — left side direct, right side inverted per MecanumDrive.rightSideMultiplier
        applyVoltageControl(
            raw[kFrontLeft],
            raw[kFrontRight] * RIGHT_SIDE,
            raw[kBackLeft],
            raw[kBackRight]  * RIGHT_SIDE
        );
    }

    /**
     * Voltage + torque control for four wheel power fractions.
     * Replaces the raw motor.set() call from MecanumDrive.
     */
    private void applyVoltageControl(double flPow, double frPow, double blPow, double brPow) {
        double batt = RobotHardware.batteryVoltage();
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

            m.set(power);

            if (m.getCurrent(CurrentUnit.AMPS) > MAX_CURRENT) {
                anyStalling = true;
            }
        }

        if (anyStalling) {
            stallCounter = 0;
            stalling = true;
        } else if (STALL_CHECK_INTERVAL > 0 && stallCounter >= STALL_CHECK_INTERVAL) {
            stalling = false;
            stallCounter = 0;
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
    }
}
