package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp
public class BasicTelepOp extends LinearOpMode {
    private DcMotorEx leftMotor, rightMotor;
    @Override
    public void runOpMode() throws InterruptedException {

        leftMotor = hardwareMap.get(DcMotorEx.class,"leftMotor");
        rightMotor = hardwareMap.get(DcMotorEx.class,"rightMotor");
        waitForStart();
        while(opModeIsActive()){
            leftMotor.setPower(-gamepad1.right_stick_y);
            rightMotor.setPower(-gamepad1.right_stick_y);
        }
    }
}
