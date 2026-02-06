package org.firstinspires.ftc.teamcode.pedroPathing.OpModes;


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
 **/

@Autonomous(name = "Auto after lookin at Burhaduniin poopy code - Blue", group = "Autonomous")
@Configurable
public class BlueCloseCOKE extends OpMode {

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
    private double target = 3275;
    private boolean turretActive = true;

    // Intake / transfer power (negative = pull balls inward)
    private double power = -1;
    private boolean shooterActive = true;

    /* ================= COMMAND SYSTEM ================= */

    private final CommandScheduler scheduler = CommandScheduler.getInstance();

    @Override
    public void init() {

        /* -------- Telemetry -------- */
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        /* -------- Path follower -------- */
        follower = AutoConstants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(144-128, 112, Math.toRadians(180)));
        paths = new Paths(follower);

        /* -------- Subsystems -------- */
        shooter = new Shooter(hardwareMap);
        turret  = new Turret(hardwareMap);
        limelight = new Limelight(hardwareMap);
        limelight.switchPipe(1); // 0 is red

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
                        new InstantCommand(() -> turretActive = true),
                        new InstantCommand(() -> shooterActive = true),
                        new InstantCommand(() -> intake.setPower(power))
                ),

                // Shoot 3 balls (feed only when path is done AND shooter is at speed)
                new WaitUntilCommand(() -> !follower.isBusy() && shooter.atTargetVelocity(target)),
                new InstantCommand(() -> intake.setPower(power)),
                new InstantCommand(() -> transfer.setPower(power)),
                new WaitCommand(1250),
                new InstantCommand(() -> transfer.setPower(0)),
//                new InstantCommand(() -> intake.setPower(0)),

                /* ================= STEP 2: GRAB CLOSE BALLS ================= */

                new InstantCommand(() -> panelsTelemetry.debug("Auto Step", "Grabbing close balls")),

                // Disable shooter to save power while intaking
                new ParallelCommandGroup(
                        new InstantCommand(() -> follower.followPath(paths.toAlignClose, true)),
                        new InstantCommand(() -> turretActive = false)
                ),
//
                new WaitUntilCommand(() -> !follower.isBusy()),
                new WaitCommand(1000),



                new ParallelCommandGroup(
                        new InstantCommand(() -> intake.setPower(power)),
                        new InstantCommand(() -> follower.followPath(paths.toGrabClose, true))
                ),

                new WaitUntilCommand(() -> !follower.isBusy()),
//                new WaitCommand(250),
//                new InstantCommand(() -> intake.setPower(0)),

                /* ================= STEP 3: SCORE CLOSE BALLS ================= */

                new InstantCommand(() -> panelsTelemetry.debug("Auto Step", "Scoring close balls")),

                new ParallelCommandGroup(
                        new InstantCommand(() -> follower.followPath(paths.toScoreClose, true)),
                        new InstantCommand(() -> turretActive = true)

                ),

                // Shoot 3 balls again
                new WaitUntilCommand(() -> !follower.isBusy() && shooter.atTargetVelocity(target)),
                new InstantCommand(() -> intake.setPower(power)),
                new InstantCommand(() -> transfer.setPower(power)),
                new WaitCommand(1300),
                new InstantCommand(() -> transfer.setPower(0)),


                /*======== STEP 3.5 : GRAB SECOND SET==============*/

                new InstantCommand(() -> panelsTelemetry.debug("Auto Step", "Grabbing close balls")),

                // Disable shooter to save power while intaking
                new ParallelCommandGroup(
                        new InstantCommand(() -> follower.followPath(paths.toAlignSecondary, true)),
                        new InstantCommand(() -> turretActive = false)

                ),

                new WaitUntilCommand(() -> !follower.isBusy()),
//                new WaitCommand(1000),

                new ParallelCommandGroup(
                        new InstantCommand(() -> intake.setPower(power)),
                        new InstantCommand(() -> follower.followPath(paths.toGrabSecondary, true))
                ),

                new WaitUntilCommand(() -> !follower.isBusy()),
//                new WaitCommand(250),
//                new InstantCommand(() -> intake.setPower(0)),

                /* ================= STEP 3.75: SCORE CLOSE BALLS ================= */

                new InstantCommand(() -> panelsTelemetry.debug("Auto Step", "Scoring close balls")),

                new ParallelCommandGroup(
                        new InstantCommand(() -> follower.followPath(paths.toScoreSecondary, true)),
                        new InstantCommand(() -> turretActive = true)
//                        new InstantCommand(() -> shooterActive = true)
                ),

                // Shoot 3 balls again
                new WaitUntilCommand(() -> !follower.isBusy() && shooter.atTargetVelocity(target)),
                new InstantCommand(() -> intake.setPower(power)),
                new InstantCommand(() -> transfer.setPower(power)),
//                new WaitCommand(50),
//                new InstantCommand(() -> transfer.setPower(0)),
//
//                new WaitUntilCommand(() -> !follower.isBusy() && shooter.atTargetVelocity(shooter.getRPM(), target)),
//                new InstantCommand(() -> intake.setPower(power)),
//                new InstantCommand(() -> transfer.setPower(power)),
//                new WaitCommand(150),
//                new InstantCommand(() -> transfer.setPower(0)),
//
//                new WaitUntilCommand(() -> !follower.isBusy() && shooter.atTargetVelocity(shooter.getRPM(), target)),
//                new InstantCommand(() -> intake.setPower(power)),
//                new InstantCommand(() -> transfer.setPower(power)),
                new WaitCommand(1350),
                new InstantCommand(() -> transfer.setPower(0)),

                /*======== STEP 3.5 : GRAB SECOND SET==============*/

                new InstantCommand(() -> panelsTelemetry.debug("Auto Step", "Grabbing close balls")),

                // Disable shooter to save power while intaking
                new ParallelCommandGroup(
                        new InstantCommand(() -> follower.followPath(paths.toAlignFinal, true)),
                        new InstantCommand(() -> turretActive = false)

                ),

                new WaitUntilCommand(() -> !follower.isBusy()),
//                new WaitCommand(1000),

                new ParallelCommandGroup(
                        new InstantCommand(() -> intake.setPower(power)),
                        new InstantCommand(() -> follower.followPath(paths.toGrabFinal, true))
                ),

                new WaitUntilCommand(() -> !follower.isBusy()),
//                new WaitCommand(250),
//                new InstantCommand(() -> intake.setPower(0)),

                /* ================= STEP 3.75: SCORE CLOSE BALLS ================= */

                new InstantCommand(() -> panelsTelemetry.debug("Auto Step", "Scoring close balls")),

                new ParallelCommandGroup(
                        new InstantCommand(() -> follower.followPath(paths.toScoreFinal, true)),
                        new InstantCommand(() -> turretActive = true)
//                        new InstantCommand(() -> shooterActive = true)
                ),

                // Shoot 3 balls again
                new WaitUntilCommand(() -> !follower.isBusy() && shooter.atTargetVelocity(target)),
                new InstantCommand(() -> intake.setPower(power)),
                new InstantCommand(() -> transfer.setPower(power)),
//                new WaitCommand(50),
//                new InstantCommand(() -> transfer.setPower(0)),
//
//                new WaitUntilCommand(() -> !follower.isBusy() && shooter.atTargetVelocity(shooter.getRPM(), target)),
//                new InstantCommand(() -> intake.setPower(power)),
//                new InstantCommand(() -> transfer.setPower(power)),
//                new WaitCommand(150),
//                new InstantCommand(() -> transfer.setPower(0)),
//
//                new WaitUntilCommand(() -> !follower.isBusy() && shooter.atTargetVelocity(shooter.getRPM(), target)),
//                new InstantCommand(() -> intake.setPower(power)),
//                new InstantCommand(() -> transfer.setPower(power)),
                new WaitCommand(1350),
                new InstantCommand(() -> transfer.setPower(0)),

                /* ================= STEP 4: PARK ================= */


                new InstantCommand(() -> panelsTelemetry.debug("Auto Step", "Parking")),
                new InstantCommand(() -> follower.followPath(paths.toPark, true)),
//                new WaitCommand(1000),

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
        limelight.update();

        /* -------- Turret control -------- */

        if(limelight.hasTarget() && gamepad2.right_stick_x == 0){
            turret.aimWithLimelight(-limelight.getTx());
        } else {
            turret.power(gamepad2.right_stick_x*0);
        }

        /* -------- Telemetry -------- */
        panelsTelemetry.debug("Limelight has Target", limelight.hasTarget());
        panelsTelemetry.debug("Shooter Active", shooterActive);
        panelsTelemetry.debug("Shooter RPM", shooter.getRPM());
        panelsTelemetry.debug("Shooter At target?", shooter.atTargetVelocity(target));
        panelsTelemetry.debug("Target RPM", target);
        panelsTelemetry.debug("Path Busy", follower.isBusy());
        panelsTelemetry.debug("Robot X", follower.getPose().getX());
        panelsTelemetry.debug("Robot Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());

        panelsTelemetry.update(telemetry);
    }

    /* ================= PATH DEFINITIONS ================= */

    public static class Paths {
        public int factor = 144;
        int factorAngle = 180;

        public PathChain ToScoreInitial;
        public PathChain toAlignClose;
        public PathChain toGrabClose;
        public PathChain toScoreClose;
        public PathChain toAlignSecondary;
        public PathChain toGrabSecondary;
        public PathChain toScoreSecondary;
        public PathChain toAlignFinal;
        public PathChain toGrabFinal;
        public PathChain toScoreFinal;
        public PathChain toPark;

        public Paths(Follower follower) {

            ToScoreInitial = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(factor - 128, 112), new Pose(factor - 90, 81)))
                    .setLinearHeadingInterpolation(180, 180)
                    .build();

            toAlignClose = follower .pathBuilder()
                    .addPath(new BezierLine( new Pose(factor - 90, 81), new Pose(factor - 93, 79)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();


            toGrabClose = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(factor - 93, 79), new Pose(factor - 134, 79)))
                    .setLinearHeadingInterpolation(180, 180)
                    .setTangentHeadingInterpolation()
                    .build();

            toScoreClose = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(factor - 134, 79), new Pose(factor - 90, 81)))
                    .setLinearHeadingInterpolation(180, 180)
                    .build();

            toAlignSecondary = follower .pathBuilder()
                    .addPath(new BezierLine( new Pose(factor - 90, 81), new Pose(factor - 93, 55)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            toGrabSecondary = follower .pathBuilder()
                    .addPath(new BezierLine( new Pose(factor - 93, 60), new Pose(factor - 138, 60)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            toScoreSecondary = follower .pathBuilder()
                    .addPath(new BezierLine( new Pose(factor - 138, 60), new Pose(factor - 90, 81)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            toAlignFinal = follower .pathBuilder()
                    .addPath(new BezierLine( new Pose(factor - 90, 81), new Pose(factor - 93, 45)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            toGrabFinal = follower .pathBuilder()
                    .addPath(new BezierLine( new Pose(factor - 93, 45), new Pose(factor - 138, 45)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            toScoreFinal = follower .pathBuilder()
                    .addPath(new BezierLine( new Pose(factor - 138, 45), new Pose(factor - 90, 81)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();


            toPark = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(factor - 90, 81), new Pose(factor - 100, 70)))
                    .setLinearHeadingInterpolation(180, 180)
                    .build();
        }
    }
}
