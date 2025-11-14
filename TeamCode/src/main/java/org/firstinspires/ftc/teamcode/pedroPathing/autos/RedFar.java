package org.firstinspires.ftc.teamcode.pedroPathing.autos;

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

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems.GateSubsystem;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems.ShooterSubsystem;

@Autonomous(name = "Red Far Auto", group = "Autonomous")
@Configurable
public class RedFar extends OpMode {

    private TelemetryManager panelsTelemetry;
    private Follower follower;
    private Paths paths;

    private IntakeSubsystem intakeSubsystem;
    private ShooterSubsystem shooterSubsystem;
    private GateSubsystem gateSubsystem;

    private final CommandScheduler scheduler = CommandScheduler.getInstance();
    private int pathState = 0;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);


        follower.setStartingPose(new Pose(88, 8, Math.toRadians(270)));

        paths = new Paths(follower);

        intakeSubsystem = new IntakeSubsystem(hardwareMap);
        shooterSubsystem = new ShooterSubsystem(hardwareMap);
        gateSubsystem = new GateSubsystem(hardwareMap);

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void start() {

        scheduler.schedule(new SequentialCommandGroup(

                // 1️⃣ Score initial preload
                new InstantCommand(() -> panelsTelemetry.debug("Auto Step", "Scoring initial preload")),
                new ParallelCommandGroup(
                        new InstantCommand(() -> follower.followPath(paths.ToScoreInitial, true)),
                        new InstantCommand(() -> shooterSubsystem.setTargetVelocity(1490))
                ),
                new InstantCommand(() -> gateSubsystem.open()),

                // First ball
                new WaitUntilCommand(() -> !follower.isBusy() && shooterSubsystem.atTargetVelocity()),
                new InstantCommand(() -> intakeSubsystem.intakeIn()),
                new WaitCommand(180),
                new InstantCommand(() -> intakeSubsystem.stop()),

                // Second ball
                new InstantCommand(() -> shooterSubsystem.setTargetVelocity(1410)),
                new WaitUntilCommand(() -> !follower.isBusy() && shooterSubsystem.atTargetVelocity()),
                new InstantCommand(() -> intakeSubsystem.intakeIn()),
                new WaitCommand(125),
                new InstantCommand(() -> intakeSubsystem.stop()),
                new WaitCommand(1000),

                // Third ball
                new InstantCommand(() -> shooterSubsystem.setTargetVelocity(1410)),
                new WaitUntilCommand(() -> !follower.isBusy() && shooterSubsystem.atTargetVelocity()),
                new InstantCommand(() -> intakeSubsystem.intakeIn()),
                new WaitCommand(1000),
                new InstantCommand(() -> intakeSubsystem.stop()),
                new InstantCommand(() -> shooterSubsystem.stop()),
                new InstantCommand(() -> gateSubsystem.close()),

                // 2️⃣ Grab first set of field balls
                new InstantCommand(() -> panelsTelemetry.debug("Auto Step", "Grabbing first set of balls")),
                new ParallelCommandGroup(
                        new InstantCommand(() -> follower.followPath(paths.toAlignClose, true)),
                        new InstantCommand(() -> gateSubsystem.close())
                ),
                new WaitUntilCommand(() -> !follower.isBusy()),
                new WaitCommand(1000),
                new ParallelCommandGroup(
                        new InstantCommand(() -> follower.followPath(paths.toGrabClose, true)),
                        new InstantCommand(() -> intakeSubsystem.intakeIn())
                ),
                new WaitUntilCommand(() -> !follower.isBusy()),
                new WaitCommand(1250),
                new InstantCommand(() -> intakeSubsystem.stop()),

                // 3️⃣ Score first grabbed balls
                new InstantCommand(() -> panelsTelemetry.debug("Auto Step", "Shooting first grabbed balls")),
                new ParallelCommandGroup(
                        new InstantCommand(() -> follower.followPath(paths.toScoreClose, true)),
                        new InstantCommand(() -> shooterSubsystem.setTargetVelocity(1465))
                ),

                new WaitUntilCommand(() -> !follower.isBusy() && shooterSubsystem.atTargetVelocity()),
                new InstantCommand(() -> gateSubsystem.open()),
                new WaitCommand(1000),
                new InstantCommand(() -> intakeSubsystem.intakeIn()),
                new WaitCommand(150),
                new InstantCommand(() -> intakeSubsystem.stop()),

                // Second shot
                new InstantCommand(() -> shooterSubsystem.setTargetVelocity(1410)),
                new WaitUntilCommand(() -> !follower.isBusy() && shooterSubsystem.atTargetVelocity()),
                new InstantCommand(() -> intakeSubsystem.intakeIn()),
                new WaitCommand(125),
                new InstantCommand(() -> intakeSubsystem.stop()),

                // Third shot
                new InstantCommand(() -> shooterSubsystem.setTargetVelocity(1400)),
                new WaitUntilCommand(() -> !follower.isBusy() && shooterSubsystem.atTargetVelocity()),
                new InstantCommand(() -> intakeSubsystem.intakeIn()),
                new WaitCommand(1000),
                new InstantCommand(() -> intakeSubsystem.stop()),
                new InstantCommand(() -> shooterSubsystem.stop()),
                new InstantCommand(() -> gateSubsystem.close()),

                // 4️⃣ Grab secondary balls
                new InstantCommand(() -> panelsTelemetry.debug("Auto Step", "Grabbing secondary balls")),
                new ParallelCommandGroup(
                        new InstantCommand(() -> follower.followPath(paths.toAlignSecondary, true)),
                        new InstantCommand(() -> gateSubsystem.close())
                ),
                new WaitUntilCommand(() -> !follower.isBusy()),
                new WaitCommand(1000),
                new ParallelCommandGroup(
                        new InstantCommand(() -> follower.followPath(paths.toGrabSecondary, true)),
                        new InstantCommand(() -> intakeSubsystem.intakeIn())
                ),
                new WaitUntilCommand(() -> !follower.isBusy()),
                new WaitCommand(1250),
                new InstantCommand(() -> intakeSubsystem.stop()),

                // Shoot secondary balls
                new InstantCommand(() -> panelsTelemetry.debug("Auto Step", "Shooting secondary balls")),
                new ParallelCommandGroup(
                        new InstantCommand(() -> follower.followPath(paths.toScoreSecondary, true)),
                        new InstantCommand(() -> shooterSubsystem.setTargetVelocity(1465))
                ),

                new WaitUntilCommand(() -> !follower.isBusy() && shooterSubsystem.atTargetVelocity()),
                new InstantCommand(() -> gateSubsystem.open()),
                new WaitCommand(1000),
                new InstantCommand(() -> intakeSubsystem.intakeIn()),
                new WaitCommand(150),
                new InstantCommand(() -> intakeSubsystem.stop()),

                new InstantCommand(() -> shooterSubsystem.setTargetVelocity(1420)),
                new WaitUntilCommand(() -> !follower.isBusy() && shooterSubsystem.atTargetVelocity()),
                new InstantCommand(() -> intakeSubsystem.intakeIn()),
                new WaitCommand(100),
                new InstantCommand(() -> intakeSubsystem.stop()),

                new InstantCommand(() -> shooterSubsystem.setTargetVelocity(1425)),
                new WaitUntilCommand(() -> !follower.isBusy() && shooterSubsystem.atTargetVelocity()),
                new InstantCommand(() -> intakeSubsystem.intakeIn()),
                new WaitCommand(1000),
                new InstantCommand(() -> intakeSubsystem.stop()),
                new InstantCommand(() -> shooterSubsystem.stop()),
                new InstantCommand(() -> gateSubsystem.close()),

                // 6️⃣ Park
                new InstantCommand(() -> panelsTelemetry.debug("Auto Step", "Parking")),
                new InstantCommand(() -> follower.followPath(paths.toPark, true)),
                new WaitCommand(1000),

                new InstantCommand(() -> {
                    intakeSubsystem.stop();
                    shooterSubsystem.stop();
                    gateSubsystem.close();
                    panelsTelemetry.debug("Auto Step", "Finished");
                })
        ));
    }

    @Override
    public void loop() {
        follower.update();
        scheduler.run();
        shooterSubsystem.update();

        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.update(telemetry);
    }


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

            ToScoreInitial = follower
                    .pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(88, 8),
                            new Pose(92, 19.481012658227854)))
                    .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(245))
                    .build();

            toAlignClose = follower
                    .pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(92, 19.481012658227854),
                            new Pose(85.89873417721519, 25)))
                    .setLinearHeadingInterpolation(Math.toRadians(245), Math.toRadians(0))
                    .build();

            toGrabClose = follower
                    .pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(85.89873417721519, 25),
                            new Pose(121, 25)))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .setTangentHeadingInterpolation()
                    .build();

            toScoreClose = follower
                    .pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(121, 25),
                            new Pose(88, 19.481012658227854)))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(245))
                    .build();

            toAlignSecondary = follower
                    .pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(92, 19.481012658227854),
                            new Pose(85.89873417721519, 49)))
                    .setLinearHeadingInterpolation(Math.toRadians(245), Math.toRadians(0))
                    .build();

            toGrabSecondary = follower
                    .pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(85.89873417721519, 49),
                            new Pose(121, 49)))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            toScoreSecondary = follower
                    .pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(121, 49),
                            new Pose(88, 19.481012658227854)))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(245))
                    .build();

            toPark = follower
                    .pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(92, 19.481012658227854),
                            new Pose(100.9367088607595, 39.98734177215191)))
                    .setLinearHeadingInterpolation(Math.toRadians(245), Math.toRadians(0))
                    .build();
        }
    }
}
