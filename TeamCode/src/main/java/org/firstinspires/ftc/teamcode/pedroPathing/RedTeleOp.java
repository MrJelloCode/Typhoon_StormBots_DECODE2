package org.firstinspires.ftc.teamcode.pedroPathing;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems.ShooterSubsystem;

@Configurable
@TeleOp(name = "Red TeleOp (Subsystems)", group = "Main")
public class RedTeleOp extends LinearOpMode {

    // Drivetrain motors
    private DcMotor frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor;

    // IMU
    private IMU imu;

    // Servo gate
    private Servo gate;

    // Subsystems
    private ShooterSubsystem shooter;
    private IntakeSubsystem intake;

    // Drive + control variables
    private double frontLeftPower, backLeftPower, frontRightPower, backRightPower, slowMode;

    // Tunables
    public static double servoPosition = 0.7;
    public static double shooterVelocity = 1250;

    @Override
    public void runOpMode() throws InterruptedException {

        // --- Hardware Mapping ---
        frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        backLeftMotor  = hardwareMap.get(DcMotor.class, "backLeftMotor");
        frontRightMotor= hardwareMap.get(DcMotor.class, "frontRightMotor");
        backRightMotor = hardwareMap.get(DcMotor.class, "backRightMotor");

        gate = hardwareMap.get(Servo.class, "Servo");

        shooter = new ShooterSubsystem(hardwareMap);
        intake = new IntakeSubsystem(hardwareMap);

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

        telemetry.addLine("Initialized — Ready to Start");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {

            // === DRIVER CONTROL ===
            double y  = -gamepad1.left_stick_y;
            double x  =  gamepad1.left_stick_x;
            double rx =  gamepad1.right_stick_x;

            if (gamepad1.start) imu.resetYaw();

            // Slow modes
            if (gamepad1.left_trigger > 0.1) slowMode = 0.4;
            else if (gamepad1.right_trigger > 0.1) slowMode = 0.2;
            else slowMode = 1.0;

            // Field-centric drive
            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
            rotX *= 1.1; // strafe fix

            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            frontLeftPower  = (rotY + rotX + rx) / denominator;
            backLeftPower   = (rotY - rotX + rx) / denominator;
            frontRightPower = (rotY - rotX - rx) / denominator;
            backRightPower  = (rotY + rotX - rx) / denominator;

            frontLeftMotor.setPower(frontLeftPower  * slowMode);
            backLeftMotor.setPower(backLeftPower    * slowMode);
            frontRightMotor.setPower(frontRightPower* slowMode);
            backRightMotor.setPower(backRightPower  * slowMode);


            // === SHOOTER CONTROL ===
            if (gamepad2.right_trigger > 0.1 && servoPosition == 0.4) {
                shooter.setTargetVelocity(shooterVelocity);
            } else {
                shooter.setTargetVelocity(0);
            }

            if (gamepad2.dpad_left) shooter.stop();
            shooter.update();


            // === INTAKE CONTROL ===
            // Manual control via left stick
            intake.setPower(-gamepad2.left_stick_y);

            // Auto-feed if shooter is up to speed
            if (gamepad2.left_trigger > 0.1 && shooter.atTargetVelocity()) {
                intake.intakeIn();
            } else if (Math.abs(gamepad2.left_stick_y) < 0.1 && gamepad2.left_trigger <= 0.1) {
                intake.stop();
            }


            // === SERVO GATE CONTROL ===
            if (gamepad2.a) servoPosition = 0.4; // Open
            if (gamepad2.b) servoPosition = 0.7; // Close
            gate.setPosition(servoPosition);


            // === TELEMETRY ===
            telemetry.addData("Shooter Target", shooter.getTargetVelocity());
            telemetry.addData("Shooter L Vel", shooter.getLeftVelocity());
            telemetry.addData("Shooter R Vel", shooter.getRightVelocity());
            telemetry.addData("Shooter Ready?", shooter.atTargetVelocity());
            telemetry.addData("Intake Power", intake.getCurrentPower());
            telemetry.addData("Servo Pos", servoPosition);
            telemetry.update();
        }
    }
}
