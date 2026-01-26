package org.firstinspires.ftc.teamcode.pedroPathing.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.pedroPathing.subsystems.*;

@TeleOp(name = "PurpleWinter")
public class PurpleWinter extends OpMode {

    /* ================= SUBSYSTEMS ================= */

    private Drivetrain drivetrain;
    private Shooter shooter;
    private Turret turret;
    private Limelight limelight;

    private DcMotorEx intakeMotor;
    private DcMotorEx transferMotor;

    /* ================= SHOOTER CONFIG ================= */

    private static final double SMART_IDLE_RPM = 800;
    private static final double DEFAULT_RPM = 1500;
    private static final double RPM_TOLERANCE = 75;
    private static final double TX_TOLERANCE = 1.0;
    private static final double MANUAL_OVERRIDE_RPM = 1500;
    private static final double SPINUP_TIME_SEC = 0.35;

    /* ================= BURST CONFIG ================= */

    private static final int BURST_COUNT = 4;
    private static final double FEED_TIME = 0.18;
    private static final double RECOVERY_TIME = 0.25;

    /* ================= MANUAL FEED CONFIG ================= */

    private static final double MANUAL_FEED_TIME = 0.20;

    /* ================= STATE ================= */

    private double targetRPM = SMART_IDLE_RPM;
    private double shooterEnableTime = 0;
    private boolean shooterSpunUp = false;

    private boolean bursting = false;
    private int shotsRemaining = 0;
    private double burstTimer = 0;
    private boolean feeding = false;

    private boolean manualFeeding = false;
    private double manualFeedStart = 0;

    private double lastKnownLLRPM = DEFAULT_RPM;

    /* ================= EMERGENCY STOP ================= */

    private boolean shooterDisabled = false;
    private boolean lastBState = false;

    private double slowMode = 1.0;
    private double slowTurn = 0.8;

    @Override
    public void init() {
        drivetrain = new Drivetrain(hardwareMap);
        shooter = new Shooter(hardwareMap);
        turret = new Turret(hardwareMap);
        limelight = new Limelight(hardwareMap);

        intakeMotor = hardwareMap.get(DcMotorEx.class, "intake");
        transferMotor = hardwareMap.get(DcMotorEx.class, "transfer");
    }

    @Override
    public void loop() {


        if (gamepad1.right_trigger > 0.1){
            slowMode = 0.5;
            slowTurn = 0.3;
        } else {
            slowMode = 1.0;
            slowTurn = 0.8;
        };

        drivetrain.fieldCentricDrive(gamepad1, slowMode, slowTurn);
        limelight.update();

        boolean shootHeld = gamepad2.left_trigger > 0.1;
        boolean manualOverride = gamepad2.y;

        /* ================= EMERGENCY STOP TOGGLE ================= */

        boolean bPressed = gamepad2.b;
        if (bPressed && !lastBState) {
            shooterDisabled = !shooterDisabled;
        }
        lastBState = bPressed;

        /* ================= TARGET RPM LOGIC ================= */

        if (shooterDisabled) {

            targetRPM = 0;
            bursting = false;
            shooterSpunUp = false;
            shooterEnableTime = 0;

        } else if (!shootHeld) {

            bursting = false;
            targetRPM = SMART_IDLE_RPM;
            shooterSpunUp = false;
            shooterEnableTime = 0;

        } else {

            if (manualOverride) {
                targetRPM = MANUAL_OVERRIDE_RPM;
            }
            else if (limelight.hasTarget()) {
                double llRPM = limelight.getRequiredShooterRPM();

                if (llRPM > 0) {
                    targetRPM = llRPM;
                    lastKnownLLRPM = llRPM;
                } else {
                    targetRPM = lastKnownLLRPM;
                }
            }
            else {
                targetRPM = lastKnownLLRPM;
            }

            if (!shooterSpunUp) {
                if (shooterEnableTime == 0) shooterEnableTime = getRuntime();
                if (getRuntime() - shooterEnableTime >= SPINUP_TIME_SEC) {
                    shooterSpunUp = true;
                }
            }
        }

        /* ================= SHOOTER UPDATE ================= */

        shooter.update(true, targetRPM);

        /* ================= READINESS CHECK ================= */

        boolean rpmReady =
                Math.abs(shooter.getRPM() - targetRPM) <= RPM_TOLERANCE;

        boolean readyToShoot =
                shootHeld && shooterSpunUp && rpmReady && !shooterDisabled;

        /* ================= BURST FIRE ================= */

        if (gamepad2.right_bumper && readyToShoot && !bursting) {
            bursting = true;
            shotsRemaining = BURST_COUNT;
            feeding = false;
            burstTimer = getRuntime();
        }

        if (bursting) {
            double elapsed = getRuntime() - burstTimer;

            if (!feeding && elapsed >= RECOVERY_TIME && shotsRemaining > 0) {
                feeding = true;
                burstTimer = getRuntime();
                shotsRemaining--;
            }

            if (feeding && elapsed >= FEED_TIME) {
                feeding = false;
                burstTimer = getRuntime();
            }

            if (shotsRemaining == 0 && !feeding) {
                bursting = false;
            }
        }

        /* ================= MANUAL FEED ================= */



        if ((gamepad2.right_trigger > 0.1) && readyToShoot && !bursting ) {
            transferMotor.setPower(-0.7);
        }


        /* ================= INTAKE ================= */

        intakeMotor.setPower(-gamepad2.left_stick_y * 0.6);

        /* ================= TURRET ================= */

        if (limelight.hasTarget() && Math.abs(gamepad2.right_stick_x) < 0.05) {
            turret.aimWithLimelight(limelight.getTx());
        } else {
            turret.power(-gamepad2.right_stick_x);
        }

        /* ================= TELEMETRY ================= */

        telemetry.addData("Shooter RPM", shooter.getRPM());
        telemetry.addData("Target RPM", targetRPM);
        telemetry.addData("Shooter Disabled", shooterDisabled);
        telemetry.addData("Shooter Ready", readyToShoot);
        telemetry.addData("Bursting", bursting);
        telemetry.update();
    }
}
