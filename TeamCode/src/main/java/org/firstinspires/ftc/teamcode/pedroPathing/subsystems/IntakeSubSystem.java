package org.firstinspires.ftc.teamcode.pedroPathing.subsystems;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Configurable
public class IntakeSubSystem {

    private DcMotorEx intakeMotor;
    private PIDFController pidf;

    // PIDF coefficients (tunable from dashboard)
    public static double P = 0.02;
    public static double I = 0.0;
    public static double D = 0.0;
    public static double F = 0.16;

    private double targetVelocity = 0;
    private double currentVelocity = 0;
    private double error = 0;

    public IntakeSubSystem(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotorEx.class, "Intake");
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        pidf = new PIDFController(P, I, D, F);
        pidf.setTolerance(100, 200); // error and derivative tolerance
    }

    // Allow OpMode to set velocity dynamically
    public void setTargetVelocity(double velocity) {
        this.targetVelocity = velocity;
        pidf.setSetPoint(velocity);
    }

    // Called periodically (each loop)
    public void update() {
        // Update PIDF constants live from dashboard
        pidf.setPIDF(P, I, D, F);
        pidf.setSetPoint(targetVelocity);

        // Read encoder velocity (ticks per second)
        currentVelocity = intakeMotor.getVelocity();

        // Compute output power
        double output = pidf.calculate(currentVelocity);

        // Save error for telemetry
        error = pidf.getPositionError();

        // Clamp to [-1, 1] for safety
        output = Math.max(-1.0, Math.min(1.0, output));

        // Apply motor power
        intakeMotor.setPower(output);
    }

    public boolean atTargetVelocity() {
        return pidf.atSetPoint();
    }

    public double getCurrentVelocity() {
        return currentVelocity;
    }

    public double getTargetVelocity() {
        return targetVelocity;
    }

    public double getError() {
        return error;
    }

    public void stop() {
        setTargetVelocity(0);
        intakeMotor.setPower(0);
    }
}
