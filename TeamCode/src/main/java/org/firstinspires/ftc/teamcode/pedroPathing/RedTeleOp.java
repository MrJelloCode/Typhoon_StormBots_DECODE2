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
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems.ShooterSubsystem;

@Configurable
@TeleOp
public class RedTeleOp extends LinearOpMode {

    // Drivetrain motors
    private DcMotor frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor;

    // Intake + servo
    private DcMotorEx intake;
    private Servo gate;

    // Subsystems
    private ShooterSubsystem shooter;

    // IMU
    private IMU imu;

    // Drive + control variables
    double frontLeftPower, backLeftPower, frontRightPower, backRightPower, slowMode;

    // Tunables
    public static double servoPosition = 0.7;
    public static double shooterVelocity = 1250;
    public static double powerFix = 0.7;

    @Override
    public void runOpMode() throws InterruptedException {

        // --- Hardware mapping ---
        frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        backLeftMotor  = hardwareMap.get(DcMotor.class, "backLeftMotor");
        frontRightMotor= hardwareMap.get(DcMotor.class, "frontRightMotor");
        backRightMotor = hardwareMap.get(DcMotor.class, "backRightMotor");

        intake = hardwareMap.get(DcMotorEx.class, "intake");
        gate = hardwareMap.get(Servo.class, "Servo");

        // Shooter subsystem handles its own motors
        shooter = new ShooterSubsystem(hardwareMap);

        // Motor directions
        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // IMU setup
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(parameters);

        telemetry.addLine("Initialized. Ready to start!");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {

            // --- DRIVER CONTROL ---
            double y  = -gamepad1.left_stick_y;
            double x  =  gamepad1.left_stick_x;
            double rx =  gamepad1.right_stick_x;

            if (gamepad1.start) imu.resetYaw();

            if (gamepad1.left_trigger > 0.1) slowMode = 0.4;
            else if (gamepad1.right_trigger > 0.1) slowMode = 0.2;
            else slowMode = 1.0;

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


            // --- SHOOTER CONTROL ---
            if (gamepad2.right_trigger > 0.1 && servoPosition == 0.4) {
                shooter.setTargetVelocity(shooterVelocity);
            } else {
                shooter.stop();
            }


            // Update shooter PIDF loop
            shooter.update();


            // --- INTAKE CONTROL ---
            intake.setPower(-gamepad2.left_stick_y * powerFix);

            if ((gamepad2.left_trigger > 0.1) && shooter.atTargetVelocity()) {
                intake.setPower(1);
            }

            // --- SERVO GATE CONTROL ---
            if (gamepad2.a) servoPosition = 0.4;  // Open
            if (gamepad2.b) servoPosition = 0.7;  // Close

            gate.setPosition(servoPosition);


            // --- TELEMETRY ---
            telemetry.addData("Shooter Target", shooter.getTargetVelocity());
            telemetry.addData("Left Vel (0)", shooter.getLeftVelocity());
            telemetry.addData("Right Vel(1)", shooter.getRightVelocity());
            telemetry.addLine();
            telemetry.addData("At Speed?", shooter.atTargetVelocity());
            telemetry.addData("Servo Pos", servoPosition);
            telemetry.update();
        }
    }
}
