package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

@TeleOp(name = "Blue TeleOp With Pedro", group = "TeleOp")
public class BlueTeleOp extends LinearOpMode {

    // --- Motors and hardware ---
    private DcMotor frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor;
    private DcMotorEx shooter0, shooter1, intake;
    private Servo gate;
    private IMU imu;

    // --- Pedro follower ---
    private Follower follower;

    // --- Control variables ---
    double frontLeftPower, backLeftPower, frontRightPower, backRightPower, slowMode;
    public static double servoPosition = 0.7, shooterVelocity = 1270, powerFix = 0.7;

    // --- TeleOp state ---
    private boolean isAutoActive = false;

    // --- Target positions ---
    private final Pose START_POSE = new Pose(26.442477876106196, 83.30973451327434, Math.toRadians(180));
    private final Pose HOME_POSE = new Pose(105.2920353982301, 32.97345132743363, Math.toRadians(180));
    private final Pose RANGE_POSE = new Pose(62.442, 82.035, Math.toRadians(-50));

    @Override
    public void runOpMode() throws InterruptedException {

        // Initialize hardware
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
        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                )
        );
        imu.initialize(parameters);

        // Initialize Pedro follower
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(START_POSE);

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {

            // Always update follower — so it tracks pose during manual driving
            follower.update();

            // Manual reset IMU
            if (gamepad1.start) imu.resetYaw();

            // --- Toggle slow mode ---
            if (gamepad1.left_trigger > 0.1 || gamepad1.right_trigger > 0.1) slowMode = 0.6;
            else slowMode = 1.0;

            // --- Button logic for auto moves ---
            if (gamepad1.a && !isAutoActive) {
                // Go to target position
                PathChain goToTarget = follower.pathBuilder()
                        .addPath(new BezierLine(follower.getPose(), RANGE_POSE))
                        .setLinearHeadingInterpolation(follower.getPose().getHeading(), RANGE_POSE.getHeading())
                        .build();

                follower.followPath(goToTarget, true);
                isAutoActive = true;
            }

            if (gamepad1.y && !isAutoActive) {
                // Go back home
                PathChain goHome = follower.pathBuilder()
                        .addPath(new BezierLine(follower.getPose(), HOME_POSE))
                        .setLinearHeadingInterpolation(follower.getPose().getHeading(), HOME_POSE.getHeading())
                        .build();

                follower.followPath(goHome, true);
                isAutoActive = true;
            }

            if (gamepad1.b && isAutoActive) {
                // Cancel auto and return to manual control
                follower.breakFollowing();
                isAutoActive = false;
            }

            // When auto finishes, return to manual
            if (isAutoActive && !follower.isBusy()) {
                isAutoActive = false;
            }

            // --- Manual drive only if auto is NOT running ---
            if (!isAutoActive) {
                double y = -gamepad1.left_stick_y;
                double x = gamepad1.left_stick_x * 1.1;
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
            }

            // --- Mechanism controls (your original logic preserved) ---
            if ((gamepad2.right_trigger > 0.1) && (servoPosition == 0.5)) {
                shooter0.setVelocity(1250);
                shooter1.setVelocity(1250);
            } else {
                shooter0.setVelocity(0);
                shooter1.setVelocity(0);
            }

            if (gamepad2.dpad_left) {
                shooter0.setVelocity(0);
                shooter1.setVelocity(0);
            }

            intake.setPower(-gamepad2.left_stick_y * powerFix);

            double avgVelocity = (shooter0.getVelocity() + shooter1.getVelocity()) / 2.0;
            if (gamepad2.left_trigger > 0.1 && Math.abs(avgVelocity - shooterVelocity) < 10) {
                intake.setPower(1);
            }

            // Open/close gate
            if (gamepad2.a) servoPosition = 0.5;
            if (gamepad2.b) servoPosition = 0.7;

            gate.setPosition(servoPosition);

            // --- Telemetry ---
            telemetry.addData("Mode", isAutoActive ? "AUTO (Pedro)" : "Manual");
            telemetry.addData("X", follower.getPose().getX());
            telemetry.addData("Y", follower.getPose().getY());
            telemetry.addData("Heading (deg)", Math.toDegrees(follower.getPose().getHeading()));
            telemetry.update();
        }
    }
}
