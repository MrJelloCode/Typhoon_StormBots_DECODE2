package org.firstinspires.ftc.teamcode.pedroPathing.subsystems;

import com.qualcomm.robotcore.hardware.*;
import com.arcrobotics.ftclib.controller.PIDFController;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants.TeleConstants;


public class Shooter {
    private DcMotorEx shooterMotor;
    private PIDFController pidf;


    public Shooter(HardwareMap hw) {
        shooterMotor = hw.get(DcMotorEx.class, "shooter");
        shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        pidf = new PIDFController(
                TeleConstants.SHOOTER_KP,
                TeleConstants.SHOOTER_KI,
                TeleConstants.SHOOTER_KD,
                TeleConstants.SHOOTER_KF
                );
    }


    public void update(boolean enabled) {
        if (!enabled) {
            shooterMotor.setPower(0);
            return;
        }


        double currentRPM = shooterMotor.getVelocity() * 60.0 / shooterMotor.getMotorType().getTicksPerRev();
        double power = pidf.calculate(currentRPM, TeleConstants.SHOOTER_TARGET_RPM);
        shooterMotor.setPower(power);
    }
}