package org.firstinspires.ftc.teamcode.pedroPathing.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.arcrobotics.ftclib.controller.PIDController;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants.TeleConstants;

/**
 * Turret subsystem with:
 * - Limelight PID tracking (tx -> 0)
 * - Soft encoder limits (wire protection)
 * - Wrap-around recovery when limits are hit
 * - Manual power override
 * - Telemetry hooks
 */
public class Turret {

    private final DcMotorEx turretMotor;
    private final PIDController pid;

    /* ================= ENCODER CONFIG ================= */

    private static final double TICKS_PER_REV = 28.0;   // motor encoder CPR
    private static final double GEAR_RATIO = 32.0;      // planetary gearbox

    /* ================= LIMITS ================= */

    private static final double MIN_ANGLE = -180.0;
    private static final double MAX_ANGLE = 180.0;
    private static final double WRAP_ZONE_DEG = 8.0;    // zone before limit to trigger wrap

    /* ================= POWER ================= */

    private static final double MAX_POWER = 0.75;       // normal PID clamp
    private static final double WRAP_POWER = 0.9;       // fast unwrap power
    private static final double MANUAL_SCALE = 0.35;    // manual stick scaling

    /* ================= STATE ================= */

    private boolean wrapping = false;
    private String wrapDirection = "NONE";

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

    /* ================= VISION AIM ================= */

    /**
     * Tracks AprilTag using Limelight tx.
     * Automatically unwraps at limits to prevent wire binding.
     */
    public void aimWithLimelight(double tx) {

        double angle = getTurretAngleDeg();

        boolean atRightLimit = angle >= (MAX_ANGLE - WRAP_ZONE_DEG);
        boolean atLeftLimit  = angle <= (MIN_ANGLE + WRAP_ZONE_DEG);

        // ---- WRAP LOGIC ----
        if (atRightLimit && tx > 0) {
            turretMotor.setPower(-WRAP_POWER);
            wrapping = true;
            wrapDirection = "RIGHT → LEFT";
            return;
        }

        if (atLeftLimit && tx < 0) {
            turretMotor.setPower(WRAP_POWER);
            wrapping = true;
            wrapDirection = "LEFT → RIGHT";
            return;
        }

        wrapping = false;
        wrapDirection = "NONE";

        // ---- DEADZONE ----
        if (Math.abs(tx) < 0.5) {
            turretMotor.setPower(0);
            return;
        }

        // ---- PID TRACKING ----
        double power = pid.calculate(tx, 0);
        power = clamp(power, -MAX_POWER, MAX_POWER);
        turretMotor.setPower(power);
    }

    /* ================= MANUAL ================= */

    /**
     * Manual driver control (right stick override)
     */
    public void power(double speed) {
        turretMotor.setPower(speed * MANUAL_SCALE);
        wrapping = false;
        wrapDirection = "MANUAL";
    }

    /**
     * Stops turret motion
     */
    public void stop() {
        turretMotor.setPower(0);
        wrapping = false;
        wrapDirection = "STOP";
    }

    /* ================= STATE / TELEMETRY ================= */

    public double getTurretAngleDeg() {
        double ticks = turretMotor.getCurrentPosition();
        double revs = ticks / (TICKS_PER_REV * GEAR_RATIO);
        return revs * 360.0;
    }

    public boolean isWrapping() {
        return wrapping;
    }

    public String getWrapDirection() {
        return wrapDirection;
    }

    /* ================= UTILS ================= */

    private double clamp(double val, double min, double max) {
        return Math.max(min, Math.min(max, val));
    }
}
