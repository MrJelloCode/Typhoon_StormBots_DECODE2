package org.firstinspires.ftc.teamcode.pedroPathing.OpModes;

import android.annotation.SuppressLint;

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

    private static final double DEFAULT_RPM = 1000;
    private static final double RPM_TOLERANCE = 75;
    private static final double TX_TOLERANCE = 1.5;
    private static final double MANUAL_OVERRIDE_RPM = 1500;
    private static final double SPINUP_TIME_SEC = 0.35;

    /* ================= BURST CONFIG ================= */

    private static final int BURST_COUNT = 4;
    private static final double FEED_TIME = 0.18;
    private static final double RECOVERY_TIME = 0.25;

    /* ================= MANUAL FEED CONFIG ================= */

    private static final double MANUAL_FEED_TIME = 0.20;

    /* ================= STATE ================= */

    private double targetRPM = 0;
    private double shooterEnableTime = 0;
    private boolean shooterSpunUp = false;

    private boolean bursting = false;
    private int shotsRemaining = 0;
    private double burstTimer = 0;
    private boolean feeding = false;

    private boolean manualFeeding = false;
    private double manualFeedStart = 0;

    private double lastKnownLLRPM = DEFAULT_RPM;

    @Override
    public void init() {
        drivetrain = new Drivetrain(hardwareMap);
        shooter = new Shooter(hardwareMap);
        turret = new Turret(hardwareMap);
        limelight = new Limelight(hardwareMap);

        intakeMotor = hardwareMap.get(DcMotorEx.class, "intake");
        transferMotor = hardwareMap.get(DcMotorEx.class, "transfer");
    }

    @SuppressLint("SuspiciousIndentation")
    @Override
    public void loop() {

        drivetrain.robotCentricDrive(gamepad1, 1.0, 0.8);
        limelight.update();

        boolean shootHeld = gamepad2.left_trigger > 0.1;
        boolean manualOverride = gamepad2.y;

        /* ================= TARGET RPM LOGIC ================= */

        if (!shootHeld) {
            shooterSpunUp = false;
            shooterEnableTime = 0;
            targetRPM = 0;
            bursting = false;
        } else {

            if (manualOverride) {
                targetRPM = MANUAL_OVERRIDE_RPM;
            }
            else if (limelight.hasTarget()) {
                double llRPM = limelight.getRequiredShooterRPM();

                if (llRPM > 0) {
                    targetRPM = llRPM;
                    lastKnownLLRPM = llRPM; // 🔒 store last valid solution
                } else {
                    targetRPM = lastKnownLLRPM;
                }
            }
            else {
                // 🚨 Defense fallback
                targetRPM = lastKnownLLRPM;
            }

            if (!shooterSpunUp) {
                if (shooterEnableTime == 0) shooterEnableTime = getRuntime();
                if (getRuntime() - shooterEnableTime >= SPINUP_TIME_SEC) {
                    shooterSpunUp = true;
                }
            }
        }

        shooter.update(shootHeld, targetRPM);

        /* ================= READINESS CHECK ================= */

        boolean rpmReady =
                Math.abs(shooter.getRPM() - targetRPM) <= RPM_TOLERANCE;

        boolean txReady =
                limelight.hasTarget() &&
                        Math.abs(limelight.getTx()) <= TX_TOLERANCE;

        boolean readyToShoot =
                shootHeld && shooterSpunUp && rpmReady;

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

        boolean manualFeedPressed = gamepad2.left_bumper;

        if (manualFeedPressed && readyToShoot && !bursting && !manualFeeding) {
            manualFeeding = true;
            manualFeedStart = getRuntime();
        }

        if (manualFeeding) {
            if (getRuntime() - manualFeedStart >= MANUAL_FEED_TIME) {
                manualFeeding = false;
            }
        }

        transferMotor.setPower(
                (feeding || manualFeeding) ? -0.7 : 0
        );

        /* ================= INTAKE ================= */

        intakeMotor.setPower(-gamepad2.left_stick_y * 0.6);

        /* ================= TURRET ================= */

        if (limelight.hasTarget() && Math.abs(gamepad2.right_stick_x) < 0.05) {
            turret.aimWithLimelight(limelight.getTx());
        } else {
            turret.power(-gamepad2.right_stick_x);
        }

        /* ================= TELEMETRY ================= */

        telemetry.addData("RPM", shooter.getRPM());
        telemetry.addData("Target RPM", targetRPM);
        telemetry.addData("Last LL RPM", lastKnownLLRPM);
        telemetry.addData("Shooter Ready", readyToShoot);
        telemetry.addData("Bursting", bursting);
        telemetry.addData("Manual Feed", manualFeeding);
        telemetry.update();
    }
}
