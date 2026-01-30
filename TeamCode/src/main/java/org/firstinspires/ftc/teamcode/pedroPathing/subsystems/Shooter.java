package org.firstinspires.ftc.teamcode.pedroPathing.subsystems;

import com.qualcomm.robotcore.hardware.*;

public class Shooter {

    private DcMotorEx shooterMotor;

    // Encoder ticks per motor revolution (NEVER gearbox)
    private static final double TICKS_PER_REV = 28.0;

    public Shooter(HardwareMap hw) {
        shooterMotor = hw.get(DcMotorEx.class, "shooter");
        shooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // REQUIRED for built-in velocity control
        shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Optional but recommended: brake gives more consistent RPM recovery
        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    /**
     * Updates shooter velocity.
     * @param enabled whether shooter is allowed to run
     * @param targetRPM desired flywheel RPM
     */
    public void update(boolean enabled, double targetRPM) {

        if (!enabled || targetRPM <= 0) {
            // Emergency stop or disabled
            shooterMotor.setPower(0);
            return;
        }

        // Convert RPM → ticks per second
        double targetTicksPerSecond = (targetRPM * TICKS_PER_REV) / 60.0;

        // REV Hub handles the velocity PID internally
        shooterMotor.setVelocity(targetTicksPerSecond);
    }

    /**
     * @return current shooter RPM
     */
    public double getRPM() {
        return shooterMotor.getVelocity() * 60.0 / TICKS_PER_REV;
    }

    /**
     * Simple velocity tolerance check
     */
    public boolean atTargetVelocity(double current, double target) {
        return Math.abs(current - target) < 20;
    }
}
