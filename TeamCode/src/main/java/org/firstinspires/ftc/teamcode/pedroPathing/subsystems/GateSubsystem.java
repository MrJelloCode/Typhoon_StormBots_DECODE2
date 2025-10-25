package org.firstinspires.ftc.teamcode.pedroPathing.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class GateSubsystem {

    private final Servo gateServo;

    // Tunable positions
    private double openPosition = 0.4;
    private double closedPosition = 0.7;

    public GateSubsystem(HardwareMap hardwareMap) {
        gateServo = hardwareMap.get(Servo.class, "Servo");
        gateServo.setPosition(closedPosition); // start closed
    }

    public void open() {
        gateServo.setPosition(openPosition);
    }

    public void close() {
        gateServo.setPosition(closedPosition);
    }

    public void toggle() {
        if (isOpen()) close();
        else open();
    }

    public boolean isOpen() {
        return Math.abs(gateServo.getPosition() - openPosition) < 0.05;
    }

    public double getPosition() {
        return gateServo.getPosition();
    }
}
