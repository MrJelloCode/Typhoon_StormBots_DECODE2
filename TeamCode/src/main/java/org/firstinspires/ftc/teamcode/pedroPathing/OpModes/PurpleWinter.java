package org.firstinspires.ftc.teamcode.pedroPathing.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.*;

import org.firstinspires.ftc.teamcode.pedroPathing.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems.Intake;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems.Turret;


@TeleOp(name = "PurpleWinter")
public class PurpleWinter extends OpMode {
    private Drivetrain drivetrain;
    private Turret turret;
    private Shooter shooter;
    private Intake intake;


    @Override
    public void init() {
        drivetrain = new Drivetrain(hardwareMap);
        turret = new Turret(hardwareMap);
        shooter = new Shooter(hardwareMap);
        intake = new Intake(hardwareMap);


        turret.zeroTurret();
    }


    @Override
    public void loop() {
        drivetrain.fieldCentricDrive(gamepad1, 1.0, 0.8);


// Example: lock turret forward on field
        if (gamepad2.a) turret.setTargetFieldAngle(0);
        if (gamepad2.b) turret.setTargetFieldAngle(90);
        if (gamepad2.x) turret.setTargetFieldAngle(-90);


        turret.update();


        shooter.update(gamepad2.right_trigger > 0.3);


        if (gamepad2.left_trigger > 0.2) intake.setPower(1.0);
        else if (gamepad2.left_bumper) intake.setPower(-1.0);
        else intake.setPower(0);
    }
}