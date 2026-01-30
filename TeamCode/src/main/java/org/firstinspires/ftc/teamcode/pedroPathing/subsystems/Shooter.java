package org.firstinspires.ftc.teamcode.pedroPathing.subsystems;

import com.qualcomm.robotcore.hardware.*;
import com.arcrobotics.ftclib.controller.PIDFController;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants.TeleConstants;

public class Shooter {
    private DcMotorEx shooterMotor;
    private PIDFController pidf;

    private static final double TICKS_PER_REV = 28.0;

    public Shooter(HardwareMap hw) {
        shooterMotor = hw.get(DcMotorEx.class, "shooter");
        shooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        pidf = new PIDFController(
                TeleConstants.SHOOTER_KP,
                TeleConstants.SHOOTER_KI,
                TeleConstants.SHOOTER_KD,
                TeleConstants.SHOOTER_KF
        );
    }

    public void update(boolean enabled, double targetRPM) {
        if (!enabled) {
            shooterMotor.setPower(0);
            return;
        }

        double currentRPM = shooterMotor.getVelocity() * 60.0 / TICKS_PER_REV;
        double power = pidf.calculate(currentRPM, targetRPM);

        shooterMotor.setPower(power);
    }

    public double getRPM() {
        return shooterMotor.getVelocity() * 60.0 / TICKS_PER_REV;
    }

    public boolean atTargetVelocity(double current, double target){
        return(Math.abs(current - target) < 20);
    };
}
