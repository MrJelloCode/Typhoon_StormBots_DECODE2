package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class HDrive {

    private DcMotorEx leftMotor;
    private DcMotorEx rightMotor;
    private DcMotorEx centerMotor;

    private IMU imu;

    private static final double COUNTS_PER_MOTOR_REV = 537.7;
    private static final double WHEEL_DIAMETER_INCHES = 3.78;

    private static final double COUNTS_PER_INCH = COUNTS_PER_MOTOR_REV / (WHEEL_DIAMETER_INCHES * Math.PI);

    public HDrive(HardwareMap hardwareMap) {

        leftMotor = hardwareMap.get(DcMotorEx.class, "leftMotor");
        rightMotor = hardwareMap.get(DcMotorEx.class, "rightMotor");
        centerMotor = hardwareMap.get(DcMotorEx.class, "centerMotor");

        imu = hardwareMap.get(IMU.class, "imu");

        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                )
        );

        imu.initialize(parameters);

        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        setBrakeMode();
    }

    public void setBrakeMode() {
        leftMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        centerMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    }

    public double getHeadingRadians() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }

    public void resetHeading() {
        imu.resetYaw();
    }

    public void stop() {
        leftMotor.setPower(0);
        rightMotor.setPower(0);
        centerMotor.setPower(0);
    }

    public void resetEncoders() {

        leftMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        centerMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        leftMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        centerMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }


    public void driveFieldOriented(double forward, double strafe, double turn) {

        double heading = getHeadingRadians();

        // Rotate joystick vector by negative heading
        double rotatedForward =
                forward * Math.cos(-heading) - strafe * Math.sin(-heading);

        double rotatedStrafe =
                forward * Math.sin(-heading) + strafe * Math.cos(-heading);

        double leftPower = rotatedForward + turn;
        double rightPower = rotatedForward - turn;
        double centerPower = rotatedStrafe;

        // Normalize powers
        double max = Math.max(
                1.0,
                Math.max(
                        Math.abs(leftPower),
                        Math.max(Math.abs(rightPower), Math.abs(centerPower))
                )
        );

        leftPower /= max;
        rightPower /= max;
        centerPower /= max;

        leftMotor.setPower(leftPower);
        rightMotor.setPower(rightPower);
        centerMotor.setPower(centerPower);
    }

    public void driveRobotOriented(double forward, double strafe, double turn){

        double left = forward + turn;
        double right = forward - turn;

        rightMotor.setPower(right);
        leftMotor.setPower(left);
        centerMotor.setPower(strafe);

    }

    public void driveDistance(double power, double inches) {

        int moveCounts = (int)(inches * COUNTS_PER_INCH);

        int leftTarget =
                leftMotor.getCurrentPosition() + moveCounts;

        int rightTarget =
                rightMotor.getCurrentPosition() + moveCounts;

        leftMotor.setTargetPosition(leftTarget);
        rightMotor.setTargetPosition(rightTarget);

        leftMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        rightMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        leftMotor.setPower(Math.abs(power));
        rightMotor.setPower(Math.abs(power));

        while (leftMotor.isBusy() && rightMotor.isBusy()) {
            // wait
        }

        stop();

        leftMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }

    public void strafeDistance(double power, double inches) {

        int moveCounts = (int)(inches * COUNTS_PER_INCH);

        int target =
                centerMotor.getCurrentPosition() + moveCounts;

        centerMotor.setTargetPosition(target);

        centerMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        centerMotor.setPower(Math.abs(power));

        while (centerMotor.isBusy()) {
            // wait
        }

        stop();

        centerMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }

    public void rotateToHeading(double power, double targetDegrees) {

        double error;

        do {

            double currentHeading =
                    Math.toDegrees(getHeadingRadians());

            error = targetDegrees - currentHeading;

            double turn =
                    Math.signum(error) * power;

            leftMotor.setPower(turn);
            rightMotor.setPower(-turn);

        } while (Math.abs(error) > 2);

        stop();
    }


}



