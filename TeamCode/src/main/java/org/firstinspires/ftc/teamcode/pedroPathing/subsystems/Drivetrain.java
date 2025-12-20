package org.firstinspires.ftc.teamcode.pedroPathing.subsystems;

import com.qualcomm.robotcore.hardware.*;
import org.firstinspires.ftc.robotcore.external.navigation.*;


public class Drivetrain {
    private DcMotorEx frontLeft, frontRight, backLeft, backRight;
    private IMU imu;


    public Drivetrain(HardwareMap hw) {
        frontLeft = hw.get(DcMotorEx.class, "frontLeft");
        frontRight = hw.get(DcMotorEx.class, "frontRight");
        backLeft = hw.get(DcMotorEx.class, "backLeft");
        backRight = hw.get(DcMotorEx.class, "backRight");


        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);


        imu = hw.get(IMU.class, "imu");
    }


    public void fieldCentricDrive(Gamepad gamepad, double slowMode, double slowTurn) {
        double y = -gamepad.left_stick_y;
        double x = gamepad.left_stick_x * 1.1;
        double rx = gamepad.right_stick_x * slowTurn;


        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
        rotX *= 1.1;


        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);


        frontLeft.setPower((rotY + rotX + rx) / denominator * slowMode);
        backLeft.setPower((rotY - rotX + rx) / denominator * slowMode);
        frontRight.setPower((rotY - rotX - rx) / denominator * slowMode);
        backRight.setPower((rotY + rotX - rx) / denominator * slowMode);
    }
}