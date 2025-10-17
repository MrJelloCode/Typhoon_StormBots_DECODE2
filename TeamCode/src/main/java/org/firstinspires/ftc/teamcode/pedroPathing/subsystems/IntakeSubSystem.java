package org.firstinspires.ftc.teamcode.pedroPathing.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class IntakeSubSystem {

    private DcMotorEx intakeMotor;
    private PIDFController pidf;

    // Tunable PIDF values (dashboard adjustable)
    public static double P = 0.02;
    public static double I = 0.0;
    public static double D = 0.0;
    public static double F = 0.16;

    // Target velocity in ticks per second
    public static double targetVelocity = 20000;

    public IntakeSubSystem(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotorEx.class, "Intake");
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        // Important: use raw encoder values (RUN_WITHOUT_ENCODER) when using custom PIDF
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Initialize FTCLib PIDF controller
        pidf = new PIDFController(P, I, D, F);
        pidf.setSetPoint(targetVelocity);

        // Optional: allow some tolerance so it doesn’t oscillate
        pidf.setTolerance(100, 200); // error tolerance and derivative tolerance
    }

    public void update() {
        // Refresh tunable constants live from the dashboard
        pidf.setPIDF(P, I, D, F);
        pidf.setSetPoint(targetVelocity);

        // Read the motor’s current velocity
        double currentVelocity = intakeMotor.getVelocity(); // ticks per second

        // Compute PIDF output
        double output = pidf.calculate(currentVelocity);

        // Clip power output to [-1, 1]
        output = Math.max(-1.0, Math.min(1.0, output));

        intakeMotor.setPower(output);
    }

    public boolean atTargetVelocity() {
        return pidf.atSetPoint();
    }

    public void stop() {
        intakeMotor.setPower(0);
    }
}
