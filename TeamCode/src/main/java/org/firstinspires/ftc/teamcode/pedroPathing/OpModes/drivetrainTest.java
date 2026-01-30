package org.firstinspires.ftc.teamcode.pedroPathing.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.pedroPathing.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems.Turret;

@TeleOp(name = "Basic")
public class drivetrainTest extends OpMode {

    /* ================= SUBSYSTEMS ================= */

    private DcMotorEx frontLeft, frontRight, backLeft, backRight;


    /* ================= EMERGENCY STOP ================= */


    private double slowMode = 1.0;
    private double slowTurn = 0.8;
    private double target = 0;
    private boolean shooterEnable = true;

    @Override
    public void init() {
        frontLeft = hardwareMap.get(DcMotorEx.class, "BR");
        frontRight = hardwareMap.get(DcMotorEx.class, "FR");
        backLeft = hardwareMap.get(DcMotorEx.class, "BL");
        backRight = hardwareMap.get(DcMotorEx.class, "FL");
    }

    @Override
    public void loop() {


            frontRight.setPower(gamepad1.right_trigger);
            frontLeft.setPower(gamepad1.left_trigger);
            backLeft.setPower(gamepad1.left_stick_y);
            backRight.setPower(gamepad2.right_stick_y);




    }
}
