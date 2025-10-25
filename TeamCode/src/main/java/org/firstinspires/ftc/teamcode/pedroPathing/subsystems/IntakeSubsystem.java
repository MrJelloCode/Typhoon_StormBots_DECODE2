package org.firstinspires.ftc.teamcode.pedroPathing.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Configurable
public class IntakeSubsystem {

    private DcMotorEx intakeMotor;

    // Tunable via dashboard for quick testing
    public static double powerMultiplier = 0.7;

    private double currentPower = 0.0;

    public IntakeSubsystem(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intake");
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }

    /** Sets intake power (automatically scaled by powerMultiplier) */
    public void setPower(double power) {
        currentPower = power * powerMultiplier;
        currentPower = Math.max(-1.0, Math.min(1.0, currentPower)); // safety clamp
        intakeMotor.setPower(currentPower);
    }

    /** Runs intake inward (for collecting game pieces) */
    public void intakeIn() {
        setPower(1.0);
    }

    /** Runs intake outward (for ejecting game pieces) */
    public void intakeOut() {
        setPower(-1.0);
    }

    /** Stops intake completely */
    public void stop() {
        setPower(0);
    }

    /** Get current applied power */
    public double getCurrentPower() {
        return currentPower;
    }
}
