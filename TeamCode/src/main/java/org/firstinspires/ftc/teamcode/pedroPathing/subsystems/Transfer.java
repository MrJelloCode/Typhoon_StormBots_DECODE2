package org.firstinspires.ftc.teamcode.pedroPathing.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class Transfer {
    private DcMotorEx transferMotor;


    public Transfer(HardwareMap hw) {
        transferMotor = hw.get(DcMotorEx.class, "transfer");
    }


    public void setPower(double power) {
        transferMotor.setPower(power);
    }
}