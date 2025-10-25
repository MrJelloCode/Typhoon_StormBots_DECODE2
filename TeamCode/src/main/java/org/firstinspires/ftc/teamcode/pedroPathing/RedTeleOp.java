package org.firstinspires.ftc.teamcode.pedroPathing;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems.GateSubsystem;

@Configurable
@TeleOp(name = "Red TeleOp", group = "Main")
public class RedTeleOp extends LinearOpMode {

    private DcMotor frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor;
    private IMU imu;

    private ShooterSubsystem shooter;
    private IntakeSubsystem intake;
    private GateSubsystem gate;

    private double frontLeftPower, backLeftPower, frontRightPower, backRightPower, slowMode;

    public static double shooterVelocity = 1250;
    public static double powerFix = 0.7;

    @Override
    public void runOpMode() throws InterruptedException {

        // Initialize drive motors
        frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        backLeftMotor  = hardwareMap.get(DcMotor.class, "backLeftMotor");
        frontRightMotor= hardwareMap.get(DcMotor.class, "frontRightMotor");
        backRightMotor = hardwareMap.get(DcMotor.class, "backRightMotor");

        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Initialize subsystems
        shooter = new ShooterSubsystem(hardwareMap);
        intake = new IntakeSubsystem(hardwareMap);
        gate = new GateSubsystem(hardwareMap);

        // IMU setup
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(parameters);

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {

            // ---------------- DRIVE ----------------
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

            // ---------------- SHOOTER ----------------
            if (gamepad2.right_trigger > 0.1 && gate.isOpen()) {
                shooter.setTargetVelocity(shooterVelocity);
            } else {
                shooter.stop();
            }

            if (gamepad2.dpad_left) shooter.stop();

            // ---------------- INTAKE ----------------
            intake.setPower(-gamepad2.left_stick_y * powerFix);

            if (gamepad2.left_trigger > 0.1 && shooter.atTargetVelocity()) {
                intake.setPower(1);
            }

            // ---------------- GATE ----------------
            if (gamepad2.a) gate.open();
            if (gamepad2.b) gate.close();

            // ---------------- TELEMETRY ----------------
            telemetry.addData("Shooter L Vel", shooter.getLeftVelocity());
            telemetry.addData("Shooter R Vel", shooter.getRightVelocity());
            telemetry.addData("Shooter Target", shooter.getTargetVelocity());
            telemetry.addData("Shooter Ready", shooter.atTargetVelocity());
            telemetry.addData("Intake Power", intake.getCurrentPower());
            telemetry.addData("Gate Pos", gate.getPosition());
            telemetry.update();
        }
    }
}
