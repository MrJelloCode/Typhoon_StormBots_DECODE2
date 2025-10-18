package org.firstinspires.ftc.teamcode.pedroPathing.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class IntakeSubSystem {

    private DcMotorEx intakeMotor;
    private PIDFController pidf;

    // PIDF constants (tunable from dashboard)
    public static double P = 0.02;
    public static double I = 0.0;
    public static double D = 0.0;
    public static double F = 0.16;

    // Default velocity (can be overridden in opmode)
    private double targetVelocity = 0;

    public IntakeSubSystem(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotorEx.class, "Intake");
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        pidf = new PIDFController(P, I, D, F);
        pidf.setTolerance(100, 200);
    }

    // --- NEW ---
    public void setTargetVelocity(double velocity) {
        this.targetVelocity = velocity;
        pidf.setSetPoint(velocity);
    }

    public void update() {
        // Refresh PIDF values
        pidf.setPIDF(P, I, D, F);

        // Read current motor velocity
        double currentVelocity = intakeMotor.getVelocity();

        // Calculate new motor output
        double output = pidf.calculate(currentVelocity);

        // Clamp between -1 and 1
        output = Math.max(-1.0, Math.min(1.0, output));

        intakeMotor.setPower(output);
    }

    public boolean atTargetVelocity() {
        return pidf.atSetPoint();
    }

    public void stop() {
        setTargetVelocity(0);
        intakeMotor.setPower(0);
    }
}
