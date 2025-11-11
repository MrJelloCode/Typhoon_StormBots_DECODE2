package org.firstinspires.ftc.teamcode.pedroPathing.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Configurable
public class ShooterSubsystem {

    private final DcMotorEx shooter0, shooter1;

    // --- Telemetry / target values ---
    private double targetVelocity = 0;   // in ticks per second
    private double leftVelocity = 0;
    private double rightVelocity = 0;

    public ShooterSubsystem(HardwareMap hardwareMap) {
        shooter0 = hardwareMap.get(DcMotorEx.class, "shooter0");
        shooter1 = hardwareMap.get(DcMotorEx.class, "shooter1");

        // Reverse one side if needed so both spin the same direction
        shooter0.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter1.setDirection(DcMotorSimple.Direction.FORWARD);

        shooter0.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        shooter1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }

    /** Directly set the shooter velocity in ticks per second. */
    public void setTargetVelocity(double velocity) {
        targetVelocity = velocity;
        shooter0.setVelocity(velocity);
        shooter1.setVelocity(velocity);
    }

    /** Stop shooter motors immediately. */
    public void stop() {
        targetVelocity = 0;
        shooter0.setPower(0);
        shooter1.setPower(0);
    }

    /** Update telemetry info (call every loop). */
    public void update() {
        leftVelocity = shooter0.getVelocity();
        rightVelocity = shooter1.getVelocity();
    }

    /** Check if both flywheels are within ±100 ticks/s of target. */
    public boolean atTargetVelocity() {
        return Math.abs(shooter0.getVelocity() - targetVelocity) < 30 && Math.abs(shooter1.getVelocity() - targetVelocity) < 30;
    }

    // --- Telemetry getters ---
    public double getLeftVelocity() { return leftVelocity; }
    public double getRightVelocity() { return rightVelocity; }
    public double getTargetVelocity() { return targetVelocity; }
}
