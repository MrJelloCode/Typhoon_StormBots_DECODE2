package org.firstinspires.ftc.teamcode.pedroPathing.OpModes;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

@TeleOp(name = "Blue TeleOp With Pedro", group = "TeleOp")
public class RedWinter extends LinearOpMode {

    private static final Logger log = LoggerFactory.getLogger(RedWinter.class);
    // --- DRIVE MOTORS ---
    private DcMotor frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor;


    private IMU imu;


    // --- CONTROL VARIABLES ---
    double frontLeftPower, backLeftPower, frontRightPower, backRightPower, slowMode;
    public static double servoPosition = 0.7, shooterVelocity = 1270, powerFix = 0.7, slowTurn;

    @Override
    public void runOpMode() throws InterruptedException {

        // =============================
        // HARDWARE INITIALIZATION
        // =============================
        frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        backLeftMotor = hardwareMap.get(DcMotor.class, "backLeftMotor");
        frontRightMotor = hardwareMap.get(DcMotor.class, "frontRightMotor");
        backRightMotor = hardwareMap.get(DcMotor.class, "backRightMotor");



        // --- Motor direction setup (important for mecanum kinematics) ---
        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);


        // --- Initialize IMU orientation ---
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                )
        );
        imu.initialize(parameters);


        waitForStart();
        if (isStopRequested()) return;

        // =============================
        // MAIN TELEOP LOOP
        // =============================
        while (opModeIsActive()) {

            // --- Manual IMU reset (useful if drift accumulates) ---
            if (gamepad1.start) imu.resetYaw();

            // --- Slow mode control for precision driving ---
            if (gamepad1.left_trigger > 0.1 || gamepad1.right_trigger > 0.1) {
                slowMode = 0.6;
                slowTurn = 0.5;
            } else {
                slowMode = 1.0;
                slowTurn = 1.0;
            }


                double y = -gamepad1.left_stick_y;   // Forward/back
                double x = gamepad1.left_stick_x * 1.1; // Strafe
                double rx = gamepad1.right_stick_x * slowTurn;  // Rotation

                // --- Field-centric drive ---
                double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
                double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
                double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
                rotX *= 1.1; // Strafe correction

                double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
                frontLeftPower = (rotY + rotX + rx) / denominator;
                backLeftPower = (rotY - rotX + rx) / denominator;
                frontRightPower = (rotY - rotX - rx) / denominator;
                backRightPower = (rotY + rotX - rx) / denominator;

                frontLeftMotor.setPower(frontLeftPower * slowMode);
                backLeftMotor.setPower(backLeftPower * slowMode);
                frontRightMotor.setPower(frontRightPower * slowMode);
                backRightMotor.setPower(backRightPower * slowMode);


            // ==================================================
            // MECHANISM CONTROLS (SHOOTER, INTAKE, SERVO)
            // ==================================================



                    frontLeftMotor.setPower(rx);
                    backLeftMotor.setPower(rx);
                    frontRightMotor.setPower(-rx);
                    backRightMotor.setPower(-rx);


        }
    }
}

