package org.firstinspires.ftc.teamcode.pedroPathing.subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.net.BindException;

/**
 * Streamlined Limelight-based shooter helper
 * Focused on geometry + hood angle + flywheel size
 */
public class Limelight {

    private Limelight3A limelight;
    private double tx, ty, ta;
    private boolean hasTarget, updating = false;

    /* =====================
       CAMERA / GEOMETRY
       ===================== */

    public double limelightHeightInches = 14.0;
    public double limelightPitchDegrees = 15.0; // camera tilt up

    public double targetHeightInches = 30.0;

    /* =====================
       SHOOTER GEOMETRY
       ===================== */

    public double shooterHeightInches = 14.0;   // separate for tuning
    public double hoodAngleDegrees = 55.0;      // ball exit angle

    /* =====================
       FLYWHEEL
       ===================== */

    public double flywheelDiameterInches = 3.0;

    /* =====================
       TUNING OFFSETS
       ===================== */

    public double distanceOffsetInches = 0.0;
    public double rpmOffset = 0.0;

    private static final double GRAVITY = 386.09; // in/s^2

    public Limelight(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.setPollRateHz(100);
        limelight.start();
    }

    public void switchPipe(int index) {
        limelight.pipelineSwitch(index);
    }

    public void update() {
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            tx = result.getTx();
            ty = result.getTy();
            ta = result.getTa();
            hasTarget = true;
        } else {
            tx = ty = ta = 0;
            hasTarget = false;
        }
        updating = true;
    }

    public boolean hasTarget() { return hasTarget; }
    public boolean isUpdating() { return updating; }

    public double getTx() { return tx; }
    public double getTy() { return ty; }
    public double getTa() { return ta; }

    /* =====================
       DISTANCE ESTIMATION
       ===================== */

    public double getDistanceInches() {
        double angleRad = Math.toRadians(limelightPitchDegrees + ty);
        if (Math.abs(Math.tan(angleRad)) < 0.01) return 0;

        double distance = (targetHeightInches - limelightHeightInches)
                / Math.tan(angleRad);

        return distance + distanceOffsetInches;
    }

    /* =====================
       BALLISTIC SOLVER
       ===================== */

    private double calculateExitVelocityIPS() {
        double d = getDistanceInches();
        if (d <= 0) return 0;

        double theta = Math.toRadians(hoodAngleDegrees);
        double deltaH = targetHeightInches - shooterHeightInches;

        double cos = Math.cos(theta);
        double tan = Math.tan(theta);

        double denominator = 2 * cos * cos * (d * tan - deltaH);
        if (denominator <= 0) return 0;

        return Math.sqrt((GRAVITY * d * d) / denominator);
    }

    /* =====================
       RPM CONVERSION
       ===================== */

    public double getRequiredShooterRPM() {
        double v = calculateExitVelocityIPS();
        if (v <= 0) return 0;

        double wheelCircumference = Math.PI * flywheelDiameterInches;
        double baseRPM = (v / wheelCircumference) * 60.0;

        return baseRPM + rpmOffset;
    }
}
