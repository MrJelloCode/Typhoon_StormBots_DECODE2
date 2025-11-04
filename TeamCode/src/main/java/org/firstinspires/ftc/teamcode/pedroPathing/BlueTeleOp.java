package org.firstinspires.ftc.teamcode.pedroPathing;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Configurable
@TeleOp
public class BlueTeleOp extends LinearOpMode {

    private DcMotor frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor;

    private DcMotorEx shooter0, shooter1, intake;
    private IMU imu;

    private Servo gate;


    double frontLeftPower, backLeftPower, frontRightPower, backRightPower, slowMode;
    public static double servoPosition = 0.7, shooterVelocity = 1250, powerFix = 0.6;





    @Override
    public void runOpMode() throws InterruptedException {

        frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        backLeftMotor  = hardwareMap.get(DcMotor.class, "backLeftMotor");
        frontRightMotor= hardwareMap.get(DcMotor.class, "frontRightMotor");
        backRightMotor = hardwareMap.get(DcMotor.class, "backRightMotor");

        shooter0 = hardwareMap.get(DcMotorEx.class, "shooter0");
        shooter1 = hardwareMap.get(DcMotorEx.class, "shooter1");
        intake = hardwareMap.get(DcMotorEx.class, "intake");

        gate = hardwareMap.get(Servo.class, "Servo");



        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        shooter0.setDirection(DcMotorSimple.Direction.REVERSE);

        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu.initialize(parameters);

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {


            if (gamepad1.start) {
                imu.resetYaw();
            }

            if (gamepad1.left_trigger > 0.1) slowMode = 0.4;
            else if (gamepad1.right_trigger > 0.1) slowMode = 0.2;
            else slowMode = 1.0;

            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
            rotX *= 1.1;

            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            frontLeftPower  = (rotY + rotX + rx) / denominator;
            backLeftPower   = (rotY - rotX + rx) / denominator;
            frontRightPower = (rotY - rotX - rx) / denominator;
            backRightPower  = (rotY + rotX - rx) / denominator;

            frontLeftMotor.setPower(frontLeftPower  * slowMode);
            backLeftMotor.setPower(backLeftPower    * slowMode);
            frontRightMotor.setPower(frontRightPower* slowMode);
            backRightMotor.setPower(backRightPower  * slowMode);

            if((gamepad2.right_trigger > 0.1) && (servoPosition == 0.4)) {
                shooter0.setVelocity(1250);
                shooter1.setVelocity(1250);
            }
            else{
                shooter0.setVelocity(0);
                shooter1.setVelocity(0);
            }
            if(gamepad2.dpad_left) {
                shooter0.setVelocity(0);
                shooter1.setVelocity(0);
            }

            intake.setPower(-gamepad2.left_stick_y*0.6);
            intake.setPower(-gamepad2.left_stick_y*powerFix);

            if((gamepad2.left_trigger > 0.1) && ((shooter1.getVelocity() - 100 <  shooterVelocity) && (shooterVelocity < shooter1.getVelocity() + 100))) {
                intake.setPower(1);
            }

            //Open
            if(gamepad2.a) {
                servoPosition = 0.4;
            }

            //Close
            if(gamepad2.b) {
                servoPosition = 0.7;
            }

            gate.setPosition(servoPosition);
        }

    }
}