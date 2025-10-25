package org.firstinspires.ftc.teamcode.pedroPathing.subsystems;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Configurable
public class ShooterSubsystem {

    // Left + Right flywheel motors
    private DcMotorEx shooter0, shooter1;

    // Independent PIDF controllers
    private PIDFController leftPIDF, rightPIDF;

    // PIDF coefficients (tunable from dashboard)
    public static double P = 0.02;
    public static double I = 0.0;
    public static double D = 0.0;
    public static double F = 0.16;

    // Targets & telemetry values
    private double targetVelocity = 0;
    private double leftVelocity = 0;
    private double rightVelocity = 0;
    private double leftError = 0;
    private double rightError = 0;

    public ShooterSubsystem(HardwareMap hardwareMap) {
        shooter0 = hardwareMap.get(DcMotorEx.class, "shooter0");
        shooter1 = hardwareMap.get(DcMotorEx.class, "shooter1");

        // Reverse one side if your shooter spins in opposite directions
        shooter0.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter1.setDirection(DcMotorSimple.Direction.FORWARD);

        shooter0.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        shooter1.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        leftPIDF = new PIDFController(P, I, D, F);
        rightPIDF = new PIDFController(P, I, D, F);

        leftPIDF.setTolerance(100, 20);
        rightPIDF.setTolerance(100, 20);
    }

    /** Set desired shooter velocity (in ticks per second) */
    public void setTargetVelocity(double velocity) {
        this.targetVelocity = velocity;
        leftPIDF.setSetPoint(velocity);
        rightPIDF.setSetPoint(velocity);
    }

    /** Update both PID loops — call this every loop */
    public void update() {
        // Update PIDF coefficients live (for dashboard tuning)
        leftPIDF.setPIDF(P, I, D, F);
        rightPIDF.setPIDF(P, I, D, F);

        // Read current flywheel speeds
        leftVelocity = shooter0.getVelocity();
        rightVelocity = shooter1.getVelocity();

        // Compute outputs
        double leftOutput = leftPIDF.calculate(leftVelocity);
        double rightOutput = rightPIDF.calculate(rightVelocity);

        leftError = leftPIDF.getPositionError();
        rightError = rightPIDF.getPositionError();

        // Clamp outputs for safety
        leftOutput = Math.max(-1.0, Math.min(1.0, leftOutput));
        rightOutput = Math.max(-1.0, Math.min(1.0, rightOutput));

        // Apply power to motors
        shooter0.setPower(leftOutput);
        shooter1.setPower(rightOutput);
    }

    /** Stop shooter */
    public void stop() {
        setTargetVelocity(0);
        shooter0.setPower(0);
        shooter1.setPower(0);
    }

    /** Telemetry accessors */
    public boolean atTargetVelocity() {
        return leftPIDF.atSetPoint() && rightPIDF.atSetPoint();
    }

    public double getLeftVelocity() {
        return leftVelocity;
    }

    public double getRightVelocity() {
        return rightVelocity;
    }

    public double getTargetVelocity() {
        return targetVelocity;
    }

    public double getLeftError() {
        return leftError;
    }

    public double getRightError() {
        return rightError;
    }
}
