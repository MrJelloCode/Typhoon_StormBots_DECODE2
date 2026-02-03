package org.firstinspires.ftc.teamcode.pedroPathing.subsystems;

import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Shooter {

    private DcMotorEx shooterMotor;
    private final ElapsedTime timer = new ElapsedTime();

    // Encoder ticks per MOTOR revolution (Ultraplanetary motor encoder)
    private static final double TICKS_PER_REV = 28.0;

    // ===== SHOOTER CONFIG =====
    // Target accel in ticks/sec^2
    // Tuned for ~500g flywheel, 6000 RPM motor
    private static final double MAX_ACCEL_TPS2 = 2200.0;

    // Velocity tolerance for "at speed" check (RPM)
    private static final double VELOCITY_TOLERANCE_RPM = 40.0;

    private double commandedVelocityTPS = 0.0;

    public Shooter(HardwareMap hw) {
        shooterMotor = hw.get(DcMotorEx.class, "shooter");
        shooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Flywheels should coast
        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        timer.reset();
    }

    /**
     * Call every loop.
     *
     * @param enabled   whether shooter is allowed to run
     * @param targetRPM desired flywheel RPM
     */
    public void update(boolean enabled, double targetRPM) {

        double dt = timer.seconds();
        timer.reset();

        if (!enabled || targetRPM <= 0) {
            commandedVelocityTPS = 0.0;
            shooterMotor.setPower(0);
            return;
        }

        // Convert RPM → ticks/sec
        double targetTPS = (targetRPM * TICKS_PER_REV) / 60.0;

        // ===== TIME-BASED ACCEL LIMITING =====
        double maxDelta = MAX_ACCEL_TPS2 * dt;

        if (commandedVelocityTPS < targetTPS) {
            commandedVelocityTPS = Math.min(
                    commandedVelocityTPS + maxDelta,
                    targetTPS
            );
        } else if (commandedVelocityTPS > targetTPS) {
            commandedVelocityTPS = Math.max(
                    commandedVelocityTPS - maxDelta,
                    targetTPS
            );
        }

        shooterMotor.setVelocity(commandedVelocityTPS);
    }

    /**
     * @return current shooter RPM
     */
    public double getRPM() {
        return shooterMotor.getVelocity() * 60.0 / TICKS_PER_REV;
    }

    /**
     * @return true if shooter is within tolerance of target
     */
    public boolean atTargetVelocity(double targetRPM) {
        return Math.abs(getRPM() - targetRPM) < VELOCITY_TOLERANCE_RPM;
    }

    /**
     * Immediately stop shooter (for safety / disable)
     */
    public void stop() {
        commandedVelocityTPS = 0.0;
        shooterMotor.setPower(0);
    }
}
