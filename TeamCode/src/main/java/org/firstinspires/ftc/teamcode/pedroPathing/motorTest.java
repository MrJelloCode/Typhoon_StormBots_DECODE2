package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class motorTest extends LinearOpMode {

    private DcMotor frontLeft, frontRight, backLeft, backRight;


    @Override

    public void runOpMode() throws InterruptedException {
        frontLeft  = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        frontRight = hardwareMap.get(DcMotor.class, "frontRightMotor");
        backLeft   = hardwareMap.get(DcMotor.class, "backLeftMotor");
        backRight  = hardwareMap.get(DcMotor.class, "backRightMotor");

        waitForStart();
        while(opModeIsActive()){
            if(gamepad1.a){
                frontLeft.setPower(1);
            }
            if(gamepad1.b){
                frontRight.setPower(1);
            }
            if(gamepad1.x){
                backLeft.setPower(1);
            }
            if(gamepad1.y){
                backRight.setPower(1);
            }
        }


    }
}
