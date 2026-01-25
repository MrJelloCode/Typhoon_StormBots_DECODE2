package org.firstinspires.ftc.teamcode.pedroPathing.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.arcrobotics.ftclib.controller.PIDController;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants.TeleConstants;

public class Turret {

    private final DcMotorEx turretMotor;
    private final PIDController pid;

    // Encoder constants
    private static final double TICKS_PER_REV = 28.0;   // motor encoder CPR
    private static final double GEAR_RATIO = 32.0;      // planetary ratio

    // Safety
    private static final double MAX_POWER = 0.75;
    private static final double MIN_ANGLE = -180.0;
    private static final double MAX_ANGLE = 180.0;

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

    /**
     * Vision-based aiming.
     * Call every loop when Limelight sees an AprilTag.
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

    /**
     * Stop turret if no target
     */
    public void stop() {
        turretMotor.setPower(0);
    }

    public void power(double speed){
        turretMotor.setPower(speed*0.35);
    }

    /**
     * Returns turret angle relative to startup (degrees)
     */
    public double getTurretAngleDeg() {
        double ticks = turretMotor.getCurrentPosition();
        double revolutions = ticks / (TICKS_PER_REV * GEAR_RATIO);
        return revolutions * 360.0;
    }

    private double clamp(double val, double min, double max) {
        return Math.max(min, Math.min(max, val));
    }
}
