package org.firstinspires.ftc.teamcode.pedroPathing.autos;

import com.arcrobotics.ftclib.command.*;
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

        // ✅ Mirrored starting pose
        follower.setStartingPose(new Pose(88, 8, Math.toRadians(90)));

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
                new InstantCommand(() -> panelsTelemetry.debug("Auto Step", "Scoring initial preload")),
                new InstantCommand(() -> gateSubsystem.close()),
                new ParallelCommandGroup(
                        new InstantCommand(() -> follower.followPath(paths.ToScoreInitial, true)),
                        new InstantCommand(() -> shooterSubsystem.setTargetVelocity(1460))
                ),
                new InstantCommand(() -> gateSubsystem.open()),

                new WaitUntilCommand(() -> !follower.isBusy() && shooterSubsystem.atTargetVelocity()),
                new InstantCommand(() -> intakeSubsystem.intakeIn()),
                new WaitCommand(500),
                new InstantCommand(() -> intakeSubsystem.stop()),

                new InstantCommand(() -> shooterSubsystem.setTargetVelocity(1460)),
                new WaitUntilCommand(() -> !follower.isBusy() && shooterSubsystem.atTargetVelocity()),
                new InstantCommand(() -> intakeSubsystem.intakeIn()),
                new WaitCommand(500),
                new InstantCommand(() -> intakeSubsystem.stop()),
                new WaitCommand(1000),

                new InstantCommand(() -> shooterSubsystem.setTargetVelocity(1440)),
                new WaitUntilCommand(() -> !follower.isBusy() && shooterSubsystem.atTargetVelocity()),
                new InstantCommand(() -> intakeSubsystem.intakeIn()),
                new WaitCommand(1000),
                new InstantCommand(() -> intakeSubsystem.stop()),
                new InstantCommand(() -> shooterSubsystem.stop()),
                new InstantCommand(() -> gateSubsystem.close()),

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
                new WaitCommand(1000),
                new InstantCommand(() -> intakeSubsystem.stop()),

                new InstantCommand(() -> panelsTelemetry.debug("Auto Step", "Shooting first grabbed balls")),
                new ParallelCommandGroup(
                        new InstantCommand(() -> follower.followPath(paths.toScoreClose, true)),
                        new InstantCommand(() -> shooterSubsystem.setTargetVelocity(1440))
                ),
                new InstantCommand(() -> gateSubsystem.open()),

                new WaitUntilCommand(() -> !follower.isBusy() && shooterSubsystem.atTargetVelocity()),
                new InstantCommand(() -> intakeSubsystem.intakeIn()),
                new WaitCommand(500),
                new InstantCommand(() -> intakeSubsystem.stop()),

                new InstantCommand(() -> shooterSubsystem.setTargetVelocity(1440)),
                new WaitUntilCommand(() -> !follower.isBusy() && shooterSubsystem.atTargetVelocity()),
                new InstantCommand(() -> intakeSubsystem.intakeIn()),
                new WaitCommand(500),
                new InstantCommand(() -> intakeSubsystem.stop()),

                new InstantCommand(() -> shooterSubsystem.setTargetVelocity(1440)),
                new WaitUntilCommand(() -> !follower.isBusy() && shooterSubsystem.atTargetVelocity()),
                new InstantCommand(() -> intakeSubsystem.intakeIn()),
                new WaitCommand(1000),
                new InstantCommand(() -> intakeSubsystem.stop()),
                new InstantCommand(() -> shooterSubsystem.stop()),
                new InstantCommand(() -> gateSubsystem.close()),

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

    // ✅ Mirrored Paths (x' = 144 - x)
    public static class Paths {
        public PathChain ToScoreInitial;
        public PathChain toAlignClose;
        public PathChain toGrabClose;
        public PathChain toScoreClose;
        public PathChain toPark;

        public Paths(Follower follower) {

            ToScoreInitial = follower
                    .pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(88, 8),
                            new Pose(81.68354430379748, 19.481012658227854)))
                    .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(240))
                    .build();

            toAlignClose = follower
                    .pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(81.68354430379748, 19.481012658227854),
                            new Pose(85.89873417721519, 34)))
                    .setLinearHeadingInterpolation(Math.toRadians(240), Math.toRadians(0))
                    .build();

            toGrabClose = follower
                    .pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(85.89873417721519, 34),
                            new Pose(132.72151898734177, 34)))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .setTangentHeadingInterpolation()
                    .build();

            toScoreClose = follower
                    .pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(132.72151898734177, 35.88607594936709),
                            new Pose(81.68354430379748, 19.481012658227854)))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(120))
                    .build();

            toPark = follower
                    .pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(81.68354430379748, 19.481012658227854),
                            new Pose(100.9367088607595, 19.481012658227854)))
                    .setLinearHeadingInterpolation(Math.toRadians(120), Math.toRadians(0))
                    .build();
        }
    }
}
