package org.firstinspires.ftc.teamcode.pedroPathing.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.arcrobotics.ftclib.controller.PIDController;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants.TeleConstants;

public class Turret {

    private final DcMotorEx turretMotor;
    private final PIDController pid;

    /* ================= ENCODER CONSTANTS ================= */

    private static final double TICKS_PER_REV = 28.0;   // motor encoder CPR
    private static final double GEAR_RATIO = 32.0;      // planetary ratio

    /* ================= SAFETY LIMITS ================= */

    private static final double MAX_POWER = 1;
    private static final double MIN_ANGLE = -180.0;
    private static final double MAX_ANGLE = 180.0;

    /* ================= CONSTRUCTOR ================= */

    public Turret(HardwareMap hw) {

        turretMotor = hw.get(DcMotorEx.class, "turret");
        turretMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        pid = new PIDController(
                TeleConstants.TURRET_KP,
                TeleConstants.TURRET_KI,
                TeleConstants.TURRET_KD
        );

        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /* ================= LIMELIGHT AIMING ================= */

    /**
     * Vision-based aiming.
     * Call every loop when Limelight sees a target.
     *
     * @param tx Limelight horizontal offset (degrees)
     */
    public void aimWithLimelight(double tx) {

        // Deadband to stop jitter
        if (Math.abs(tx) < 0.5) {
            turretMotor.setPower(0);
            return;
        }

        double power = pid.calculate(tx, 0);

        // Clamp power
        power = clamp(power, -MAX_POWER, MAX_POWER);

        // Wire-wrap protection
        double angle = getTurretAngleDeg();
        if ((angle <= MIN_ANGLE && power < 0) ||
                (angle >= MAX_ANGLE && power > 0)) {
            power = 0;
        }

        turretMotor.setPower(power);
    }

    /* ================= MANUAL / SAFETY ================= */

    /**
     * Stop turret movement
     */
    public void stop() {
        turretMotor.setPower(0);
    }
    /**
     * Manual power control (scaled)
     */
    public void power(double speed) {
        turretMotor.setPower(speed);
    }

    /* ================= POSITION ================= */

    /**
     * Returns turret angle relative to startup (degrees)
     */
    public double getTurretAngleDeg() {
        double ticks = turretMotor.getCurrentPosition();
        double revolutions = ticks / (TICKS_PER_REV * GEAR_RATIO);
        return revolutions * 360.0;
    }

    /* ================= UTIL ================= */

    private double clamp(double val, double min, double max) {
        return Math.max(min, Math.min(max, val));
    }
}
