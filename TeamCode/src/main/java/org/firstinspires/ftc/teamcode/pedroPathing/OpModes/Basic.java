package org.firstinspires.ftc.teamcode.pedroPathing.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.pedroPathing.subsystems.*;

@TeleOp(name = "Basic")
public class Basic extends OpMode {

    /* ================= SUBSYSTEMS ================= */

    private Drivetrain drivetrain;
    private Shooter shooter;
    private Turret turret;
    private Limelight limelight;

    private DcMotorEx intakeMotor;
    private DcMotorEx transferMotor;


    /* ================= EMERGENCY STOP ================= */


    private double slowMode = 1.0;
    private double slowTurn = 0.8;
    private double target = 0;
    private boolean shooterEnable = true;

    @Override
    public void init() {
        drivetrain = new Drivetrain(hardwareMap);
        shooter = new Shooter(hardwareMap);
        turret = new Turret(hardwareMap);
        limelight = new Limelight(hardwareMap);

        intakeMotor = hardwareMap.get(DcMotorEx.class, "intake");
        transferMotor = hardwareMap.get(DcMotorEx.class, "transfer");
    }

    @Override
    public void loop() {


        if (gamepad1.right_trigger > 0.1){
            slowMode = 0.5;
            slowTurn = 0.3;
        } else {
            slowMode = 1.0;
            slowTurn = 0.8;
        };

        drivetrain.fieldCentricDrive(gamepad1, slowMode, slowTurn);

        intakeMotor.setPower(gamepad2.left_stick_y);
        transferMotor.setPower(-gamepad2.right_trigger*0.7);


        if(limelight.hasTarget() && gamepad2.right_stick_x == 0){
            turret.aimWithLimelight(limelight.getTx());
        } else {
            turret.power(gamepad2.right_stick_x*0.5);
        }

        if(gamepad2.a) {target = 3000; shooterEnable = true;}
        if(gamepad2.b) {target = 4200;shooterEnable = true;}
        if(gamepad2.y) {target = 0;shooterEnable = false;}

       shooter.update(shooterEnable, target);
       limelight.update();

        telemetry.addData("velocity", shooter.getRPM());
        telemetry.addData("Turret has Target", limelight.hasTarget());
        telemetry.addData("Tx:", limelight.getTx());
        telemetry.update();

    }
}
