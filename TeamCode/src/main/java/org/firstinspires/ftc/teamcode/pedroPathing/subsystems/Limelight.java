package org.firstinspires.ftc.teamcode.pedroPathing.subsystems;


import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

//Limelight Class
public class Limelight {

    //Creates limelight and limelight variables
    private Limelight3A limelight;
    private double tx, ty, ta;
    private boolean hasTarget;

    // Initializes those variables (Init)
    public Limelight(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.setPollRateHz(100);
        limelight.start();
    }

    //Loop to update all the limelight values
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

    //Limelight values return methods
    public double getTx() { return tx; }
    public double getTy() { return ty; }
    public double getTa() { return ta; }
    public boolean hasTarget() { return hasTarget; }
}
