package org.firstinspires.ftc.teamcode.pedroPathing;


import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants.AutoConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems.Turret;

/**
 * ===============================
 * Red Far Autonomous
 * ===============================
 *
 * High-level goals of this auto:
 * 1. Drive to initial scoring position and shoot preload
 * 2. Intake close field balls
 * 3. Return and score them
 * 4. Intake secondary balls
 * 5. Return and score again
 * 6. Park
 *
 * Design philosophy:
 * - Shooter PID is ENABLED only when we intend to shoot
 * - Shooter PID is DISABLED while intaking to reduce battery draw
 * - All timing is gated on path completion + shooter velocity readiness
 * - Turret tracks vision target when available, otherwise stays idle
 */
@Autonomous(name = "Red Far Auto", group = "Autonomous")
@Configurable
public class RedFar extends OpMode {

    /* ================= TELEMETRY ================= */

    private TelemetryManager panelsTelemetry;

    /* ================= PATH FOLLOWING ================= */

    private Follower follower;
    private Paths paths;

    /* ================= SUBSYSTEMS ================= */

    private Limelight limelight;
    private Shooter shooter;
    private Turret turret;

    /* ================= HARDWARE ================= */

    private DcMotorEx intake;
    private DcMotorEx transfer;

    /* ================= SHOOTER CONFIG ================= */

    // Fixed RPM target for this auto (tuned value)
    private double target = 3900;

    // Intake / transfer power (negative = pull balls inward)
    private double power = -1;

    // Master enable for shooter PID loop
    // True  = shooter actively controls RPM
    // False = shooter coasts to reduce power draw
    private boolean shooterActive = true;

    /* ================= COMMAND SYSTEM ================= */

    private final CommandScheduler scheduler = CommandScheduler.getInstance();

    @Override
    public void init() {

        /* -------- Telemetry -------- */
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        /* -------- Path follower -------- */
        follower = AutoConstants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(88, 8, Math.toRadians(0)));
        paths = new Paths(follower);

        /* -------- Subsystems -------- */
        shooter = new Shooter(hardwareMap);
        turret  = new Turret(hardwareMap);
        limelight = new Limelight(hardwareMap);

        /* -------- Motors -------- */
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        transfer = hardwareMap.get(DcMotorEx.class, "transfer");

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void start() {

        /*
         * Entire autonomous routine is defined as ONE sequential command group.
         * Each step only advances when its conditions are satisfied.
         */
        scheduler.schedule(new SequentialCommandGroup(

                /* ================= STEP 1: SCORE PRELOAD ================= */

                new InstantCommand(() -> panelsTelemetry.debug("Auto Step", "Scoring initial preload")),

                // Drive to scoring position while spinning shooter and intake
                new ParallelCommandGroup(
                        new InstantCommand(() -> follower.followPath(paths.ToScoreInitial, true)),
                        new InstantCommand(() -> shooterActive = true),
                        new InstantCommand(() -> intake.setPower(power))
                ),

                // Shoot 3 balls (feed only when path is done AND shooter is at speed)
                new WaitUntilCommand(() -> !follower.isBusy() && shooter.atTargetVelocity(target)),
                new InstantCommand(() -> intake.setPower(power)),
                new InstantCommand(() -> transfer.setPower(power)),
                new WaitCommand(50),
                new InstantCommand(() -> transfer.setPower(0)),

                new WaitUntilCommand(() -> !follower.isBusy() && shooter.atTargetVelocity(target)),
                new InstantCommand(() -> intake.setPower(power)),
                new InstantCommand(() -> transfer.setPower(power)),
                new WaitCommand(150),
                new InstantCommand(() -> transfer.setPower(0)),

                new WaitUntilCommand(() -> !follower.isBusy() && shooter.atTargetVelocity( target)),
                new InstantCommand(() -> intake.setPower(power)),
                new InstantCommand(() -> transfer.setPower(power)),
                new WaitCommand(250),
                new InstantCommand(() -> transfer.setPower(0)),
                new InstantCommand(() -> intake.setPower(0)),

                /* ================= STEP 2: GRAB CLOSE BALLS ================= */

                new InstantCommand(() -> panelsTelemetry.debug("Auto Step", "Grabbing close balls")),

                // Disable shooter to save power while intaking
                new ParallelCommandGroup(
                        new InstantCommand(() -> follower.followPath(paths.toAlignClose, true)),
                        new InstantCommand(() -> intake.setPower(0)),
                        new InstantCommand(() -> shooterActive = false)
                ),

                new WaitUntilCommand(() -> !follower.isBusy()),
                new WaitCommand(1000),

                new ParallelCommandGroup(
                        new InstantCommand(() -> intake.setPower(power)),
                        new InstantCommand(() -> follower.followPath(paths.toGrabClose, true))
                ),

                new WaitUntilCommand(() -> !follower.isBusy()),
                new WaitCommand(1000),
                new InstantCommand(() -> intake.setPower(0)),

                /* ================= STEP 3: SCORE CLOSE BALLS ================= */

                new InstantCommand(() -> panelsTelemetry.debug("Auto Step", "Scoring close balls")),

                new ParallelCommandGroup(
                        new InstantCommand(() -> follower.followPath(paths.toScoreClose, true)),
                        new InstantCommand(() -> shooterActive = true)
                ),

                // Shoot 3 balls again
                new WaitUntilCommand(() -> !follower.isBusy() && shooter.atTargetVelocity(target)),
                new InstantCommand(() -> intake.setPower(power)),
                new InstantCommand(() -> transfer.setPower(power)),
                new WaitCommand(50),
                new InstantCommand(() -> transfer.setPower(0)),

                new WaitUntilCommand(() -> !follower.isBusy() && shooter.atTargetVelocity( target)),
                new InstantCommand(() -> intake.setPower(power)),
                new InstantCommand(() -> transfer.setPower(power)),
                new WaitCommand(150),
                new InstantCommand(() -> transfer.setPower(0)),

                new WaitUntilCommand(() -> !follower.isBusy() && shooter.atTargetVelocity(target)),
                new InstantCommand(() -> intake.setPower(power)),
                new InstantCommand(() -> transfer.setPower(power)),
                new WaitCommand(350),
                new InstantCommand(() -> transfer.setPower(0)),
                new InstantCommand(() -> intake.setPower(0)),

                /* ================= STEP 4: PARK ================= */

                new InstantCommand(() -> panelsTelemetry.debug("Auto Step", "Parking")),
                new InstantCommand(() -> follower.followPath(paths.toPark, true)),
                new WaitCommand(1000),

                /* ================= FINISH ================= */

                new InstantCommand(() -> {
                    intake.setPower(0);
                    transfer.setPower(0);
                    shooter.update(false, 0);
                    panelsTelemetry.debug("Auto Step", "Finished");
                })
        ));
    }

    @Override
    public void loop() {

        /* -------- Core updates -------- */
        follower.update();
        scheduler.run();

        /* -------- Shooter control -------- */
        // Shooter PID only runs when shooterActive == true
        shooter.update(shooterActive, target);

        /* -------- Turret control -------- */
        if (limelight.hasTarget()) {
            turret.aimWithLimelight(limelight.getTx());
        } else {
            turret.stop();
        }

        /* -------- Telemetry -------- */
        panelsTelemetry.debug("Shooter Active", shooterActive);
        panelsTelemetry.debug("Shooter RPM", shooter.getRPM());
        panelsTelemetry.debug("Target RPM", target);
        panelsTelemetry.debug("Path Busy", follower.isBusy());
        panelsTelemetry.debug("Robot X", follower.getPose().getX());
        panelsTelemetry.debug("Robot Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.update(telemetry);
    }

    /* ================= PATH DEFINITIONS ================= */

    public static class Paths {
        public PathChain ToScoreInitial;
        public PathChain toAlignClose;
        public PathChain toGrabClose;
        public PathChain toScoreClose;
        public PathChain toAlignSecondary;
        public PathChain toGrabSecondary;
        public PathChain toScoreSecondary;
        public PathChain toPark;

        public Paths(Follower follower) {

            ToScoreInitial = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(88, 8), new Pose(88, 23)))
                    .setLinearHeadingInterpolation(0, 0)
                    .build();

            toAlignClose = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(88, 23), new Pose(88, 35)))
                    .setLinearHeadingInterpolation(0, 0)
                    .build();

            toGrabClose = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(88, 35), new Pose(135, 35)))
                    .setLinearHeadingInterpolation(0, 0)
                    .setTangentHeadingInterpolation()
                    .build();

            toScoreClose = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(135, 35), new Pose(88, 24)))
                    .setLinearHeadingInterpolation(0, 0)
                    .build();
            toAlignSecondary = follower .pathBuilder()
                    .addPath(new BezierLine( new Pose(88, 24), new Pose(88, 55)))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            toGrabSecondary = follower .pathBuilder()
                    .addPath(new BezierLine( new Pose(88, 55), new Pose(130, 55)))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            toScoreSecondary = follower .pathBuilder()
                    .addPath(new BezierLine( new Pose(130, 55), new Pose(88, 21)))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            toPark = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(88, 21), new Pose(88, 45)))
                    .setLinearHeadingInterpolation(0, 0)
                    .build();
        }
    }
}
