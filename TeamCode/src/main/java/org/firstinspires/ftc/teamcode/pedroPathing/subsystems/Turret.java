package org.firstinspires.ftc.teamcode.pedroPathing.subsystems;

import com.qualcomm.robotcore.hardware.*;
import com.arcrobotics.ftclib.controller.PIDController;
import org.firstinspires.ftc.robotcore.external.navigation.*;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants.TeleConstants;


public class Turret {
    private DcMotorEx turretMotor;
    private IMU imu;
    private PIDController pid;


    private static final double TICKS_PER_REV = 537.6; // goBILDA / REV planetary motor encoder
    private static final double GEAR_RATIO = 16.0; // 16:1 planetary gearbox // CHANGE


    private double targetFieldAngleDeg = 0.0;
    private double turretZeroOffsetDeg = 0.0;


    public Turret(HardwareMap hw) {
        turretMotor = hw.get(DcMotorEx.class, "turret");
        imu = hw.get(IMU.class, "imu");


        pid = new PIDController(
                TeleConstants.TURRET_KP,
                TeleConstants.TURRET_KI,
                TeleConstants.TURRET_KD
        );


        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }


    public void zeroTurret() {
        turretZeroOffsetDeg = getTurretAngleDeg();
    }


    public void setTargetFieldAngle(double angleDeg) {
        targetFieldAngleDeg = normalize(angleDeg);
    }


    public void update() {
        double robotYawDeg = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        double desiredTurretAngleDeg = normalize(targetFieldAngleDeg - robotYawDeg);


        desiredTurretAngleDeg = clamp(
                desiredTurretAngleDeg,
                TeleConstants.TURRET_MIN_ANGLE,
                TeleConstants.TURRET_MAX_ANGLE
        );


        double currentTurretAngleDeg = getTurretAngleDeg();
        double error = normalize(desiredTurretAngleDeg - currentTurretAngleDeg);


        double power = pid.calculate(error, 0);
        turretMotor.setPower(power);
    }


    private double getTurretAngleDeg() {
        double ticks = turretMotor.getCurrentPosition();
        double revs = ticks / (TICKS_PER_REV * GEAR_RATIO);
        return normalize(revs * 360.0 - turretZeroOffsetDeg);
    }


    private double normalize(double angle) {
        while (angle > 180) angle -= 360;
        while (angle < -180) angle += 360;
        return angle;
    }


    private double clamp(double val, double min, double max) {
        return Math.max(min, Math.min(max, val));
    }
}