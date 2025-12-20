package org.firstinspires.ftc.teamcode.pedroPathing.subsystems;

import com.qualcomm.robotcore.hardware.*;


public class Intake {
    private DcMotorEx intakeMotor;


    public Intake(HardwareMap hw) {
        intakeMotor = hw.get(DcMotorEx.class, "intake");
    }


    public void setPower(double power) {
        intakeMotor.setPower(power);
    }
}