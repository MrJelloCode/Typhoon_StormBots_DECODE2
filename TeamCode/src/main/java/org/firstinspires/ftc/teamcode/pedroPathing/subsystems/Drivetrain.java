package org.firstinspires.ftc.teamcode.pedroPathing.subsystems;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.*;
import org.firstinspires.ftc.robotcore.external.navigation.*;


public class Drivetrain {
    private DcMotorEx frontLeft, frontRight, backLeft, backRight;
    private IMU imu;


    public Drivetrain(HardwareMap hw) {
        frontLeft = hw.get(DcMotorEx.class, "FL");
        frontRight = hw.get(DcMotorEx.class, "FR");
        backLeft = hw.get(DcMotorEx.class, "BL");
        backRight = hw.get(DcMotorEx.class, "BR");


        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);


        imu = hw.get(IMU.class, "imu");

        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));
        imu.initialize(parameters);
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

    public void robotCentricDrive(Gamepad gamepad, double slowMode, double slowTurn){

        double y = -gamepad.left_stick_y; // Remember, Y stick value is reversed
        double x = gamepad.left_stick_x * 1.1; // Counteract imperfect strafing
        double rx = gamepad.right_stick_x;

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        frontLeft.setPower(frontLeftPower);
        backLeft.setPower(backLeftPower);
        frontRight.setPower(frontRightPower);
        backRight.setPower(backRightPower);

    }


}