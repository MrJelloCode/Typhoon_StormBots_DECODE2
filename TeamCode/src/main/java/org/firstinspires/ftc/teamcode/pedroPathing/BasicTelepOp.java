package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class BasicTelepOp extends LinearOpMode {
    private DcMotorEx leftMotor, centerMotor, rightMotor;
    @Override
    public void runOpMode() throws InterruptedException {

        leftMotor = hardwareMap.get(DcMotorEx.class,"leftMotor");
        rightMotor = hardwareMap.get(DcMotorEx.class,"rightMotor");
        centerMotor= hardwareMap.get(DcMotorEx.class,"centerMotor");

        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);


        waitForStart();

        while(opModeIsActive()){

            double speed = -gamepad1.left_stick_y;
            double turn = gamepad1.right_stick_x;

            double left = speed + turn;
            double right = speed - turn;

            rightMotor.setPower(right);
            leftMotor.setPower(left);
            centerMotor.setPower(gamepad1.left_stick_x);



        }
    }
}
