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

    // --- DRIVE MOTORS ---
    private DcMotor frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor;

    // --- MECHANISMS ---
    private DcMotorEx shooter0, shooter1, intake;
    private Servo gate;
    private IMU imu;

    // --- PEDRO FOLLOWER (handles pose tracking + path following) ---
    private Follower follower;

    // --- CONTROL VARIABLES ---
    double frontLeftPower, backLeftPower, frontRightPower, backRightPower, slowMode;
    public static double servoPosition = 0.7, shooterVelocity = 1270, powerFix = 0.7, slowTurn;

    // --- STATE FLAGS ---
    private boolean isAutoActive = false;  // True while robot is following a path

    // --- PREDEFINED FIELD POSITIONS ---
    private final Pose START_POSE = new Pose(26.44, 83.31, Math.toRadians(180));
    private final Pose HOME_POSE = new Pose(105.29, 32.97, Math.toRadians(180));
    private final Pose RANGE_POSE = new Pose(62.44, 82.04, Math.toRadians(-50));

    @Override
    public void runOpMode() throws InterruptedException {

        // =============================
        // HARDWARE INITIALIZATION
        // =============================
        frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        backLeftMotor  = hardwareMap.get(DcMotor.class, "backLeftMotor");
        frontRightMotor= hardwareMap.get(DcMotor.class, "frontRightMotor");
        backRightMotor = hardwareMap.get(DcMotor.class, "backRightMotor");

        shooter0 = hardwareMap.get(DcMotorEx.class, "shooter0");
        shooter1 = hardwareMap.get(DcMotorEx.class, "shooter1");
        intake   = hardwareMap.get(DcMotorEx.class, "intake");
        gate     = hardwareMap.get(Servo.class, "Servo");

        // --- Motor direction setup (important for mecanum kinematics) ---
        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        shooter0.setDirection(DcMotorSimple.Direction.REVERSE);

        // --- Initialize IMU orientation ---
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                )
        );
        imu.initialize(parameters);

        // =============================
        // PEDRO FOLLOWER INITIALIZATION
        // =============================
        follower = Constants.createFollower(hardwareMap);  // Your custom setup method
        follower.setStartingPose(START_POSE);              // Define starting position
        // Pedro now continuously estimates the robot’s pose (x, y, heading)

        // =============================
        // WAIT FOR START
        // =============================
        waitForStart();
        if (isStopRequested()) return;

        // =============================
        // MAIN TELEOP LOOP
        // =============================
        while (opModeIsActive()) {

            // --- Update Pedro follower every loop ---
            // Even when driving manually, this keeps tracking robot position.
            follower.update();

            // --- Manual IMU reset (useful if drift accumulates) ---
            if (gamepad1.start) imu.resetYaw();

            // --- Slow mode control for precision driving ---
            if (gamepad1.left_trigger > 0.1 || gamepad1.right_trigger > 0.1){
                slowMode = 0.6;
                slowTurn = 0.5;
            }

            else {
                slowMode = 1.0;
                slowTurn = 1.0;
            }

            // ==================================================
            // PATH TRIGGER BUTTONS (AUTO MOVEMENT)
            // ==================================================

            // (A) → Go to RANGE_POSE (e.g., shooting position)
            if (gamepad1.a && !isAutoActive) {
                PathChain goToTarget = follower.pathBuilder()
                        .addPath(new BezierLine(follower.getPose(), RANGE_POSE))
                        .setLinearHeadingInterpolation(follower.getPose().getHeading(), RANGE_POSE.getHeading())
                        .build();

                follower.followPath(goToTarget, true);
                isAutoActive = true;
            }

            // (Y) → Go back to HOME_POSE
            if (gamepad1.y && !isAutoActive) {
                PathChain goHome = follower.pathBuilder()
                        .addPath(new BezierLine(follower.getPose(), HOME_POSE))
                        .setLinearHeadingInterpolation(follower.getPose().getHeading(), HOME_POSE.getHeading())
                        .build();

                follower.followPath(goHome, true);
                isAutoActive = true;
            }

            // (B) → Cancel auto and return to manual control
            if (gamepad1.b && isAutoActive) {
                follower.breakFollowing();   // Stops Pedro immediately
                isAutoActive = false;
            }

            // Automatically exit auto once the path completes
            if (isAutoActive && !follower.isBusy()) {
                isAutoActive = false;
            }

            // ==================================================
            // MANUAL DRIVE CONTROL (only when auto not active)
            // ==================================================
            if (!isAutoActive) {
                double y = -gamepad1.left_stick_y ;   // Forward/back
                double x = gamepad1.left_stick_x * 1.1; // Strafe
                double rx = gamepad1.right_stick_x * slowTurn;  // Rotation

                // --- Field-centric drive ---
                double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
                double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
                double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
                rotX *= 1.1; // Strafe correction

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

            // ==================================================
            // MECHANISM CONTROLS (SHOOTER, INTAKE, SERVO)
            // ==================================================

            // --- SHOOTER CONTROL ---
            if ((gamepad2.right_trigger > 0.1) && (servoPosition == 0.5)) {
                shooterVelocity = 1250; // Standard shot
            } else if (-gamepad2.right_stick_y > 0 && (servoPosition == 0.5)) {
                shooterVelocity = 1450; // Power shot
            } else {
                shooterVelocity = 0; // Stop shooter
            }

            shooter0.setVelocity(shooterVelocity);
            shooter1.setVelocity(shooterVelocity);

            // --- INTAKE CONTROL ---
            intake.setPower(-gamepad2.left_stick_y * powerFix);

            // Feed only when shooters are up to speed
            double avgVelocity = (shooter0.getVelocity() + shooter1.getVelocity()) / 2.0;
            if ((gamepad2.left_trigger > 0.1)
                    && (Math.abs(avgVelocity - shooterVelocity) < 10)
                    && (avgVelocity > 100)) {
                intake.setPower(1);
            }

            // --- SERVO CONTROL (GATE OPEN/CLOSE) ---
            if (gamepad2.a) servoPosition = 0.5; // Open gate
            if (gamepad2.b) servoPosition = 0.7; // Close gate
            gate.setPosition(servoPosition);

            // ==================================================
            // TELEMETRY
            // ==================================================
            telemetry.addData("Mode", isAutoActive ? "AUTO (Pedro)" : "Manual");
            telemetry.addData("Pose X", follower.getPose().getX());
            telemetry.addData("Pose Y", follower.getPose().getY());
            telemetry.addData("Heading (deg)", Math.toDegrees(follower.getPose().getHeading()));
            telemetry.addData("Shooter Velocity", avgVelocity);
            telemetry.addData("Target Velocity", shooterVelocity);
            telemetry.update();
        }
    }
}
