package org.firstinspires.ftc.teamcode.pedroPathing.subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Limelight {

    private Limelight3A limelight;
    private double tx, ty, ta;
    private boolean hasTarget;

    // 🔧 Tunable Constants
    public double targetHeightInches = 51.0;    // Height of target (in)
    public double shooterHeightInches = 14.0;   // Height of shooter (in)
    public double shooterAngleDegrees = 45.0;   // Angle of shooter
    public double aimOffsetInches = 10.0;       // Aim above target (in)
    public double flywheelDiameterInches = 4.0; // Shooter flywheel diameter (in)
    public double distanceOffsetInches = 0.0;   // For tuning (to adjust distance calc)

    private static final double GRAVITY = 386.09; // in/s²

    public Limelight(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.setPollRateHz(100);
        limelight.start();
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
    }

    public double getTx() { return tx; }
    public double getTy() { return ty; }
    public double getTa() { return ta; }
    public boolean hasTarget() { return hasTarget; }

    // Estimate distance to target using Limelight vertical angle (ty)
    public double getDistanceInches() {
        // convert total angle to radians
        double totalAngle = Math.toRadians(ty + shooterAngleDegrees);
        return ((targetHeightInches - shooterHeightInches) / Math.tan(totalAngle)) + distanceOffsetInches;
    }

    // Calculate required shooter velocity (in inches per second)
    public double calculateRequiredVelocity() {
        double distance = getDistanceInches();
        double theta = Math.toRadians(shooterAngleDegrees);

        double deltaH = (targetHeightInches + aimOffsetInches) - shooterHeightInches;

        // avoid divide-by-zero cases
        if (distance <= 0 || Math.abs(theta) < 0.01) {
            return 0;
        }

        double numerator = GRAVITY * Math.pow(distance, 2);
        double denominator = 2 * Math.pow(Math.cos(theta), 2) * (distance * Math.tan(theta) - deltaH);

        if (denominator <= 0) {
            return 0;
        }

        double v = Math.sqrt(numerator / denominator);
        return v; // inches per second
    }

    // Convert linear velocity to shooter RPM
    public double getRequiredShooterRPM() {
        double v = calculateRequiredVelocity();
        double wheelCircumference = Math.PI * flywheelDiameterInches;
        double revsPerSecond = v / wheelCircumference;
        return revsPerSecond * 60.0; // RPM
    }
}
