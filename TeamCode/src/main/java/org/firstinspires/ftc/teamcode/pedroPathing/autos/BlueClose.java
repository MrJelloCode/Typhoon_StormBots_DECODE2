package org.firstinspires.ftc.teamcode.pedroPathing.autos;

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
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems.GateSubsystem;
import org.firstinspires.ftc.teamcode.pedroPathing.commands.RunIntakeCMD;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.arcrobotics.ftclib.command.WaitCommand;

@Autonomous(name = "Blue Close Auto", group = "Autonomous")
@Configurable
public class BlueClose extends OpMode {

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
        follower.setStartingPose(new Pose(22.329, 124.063, Math.toRadians(323)));

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

                // 🟦 1️⃣ Score initial preload
                new InstantCommand(() -> panelsTelemetry.debug("Auto Step", "Scoring initial preload")),
                new InstantCommand(() -> gateSubsystem.close()),
                new ParallelCommandGroup(
                        new InstantCommand(() -> follower.followPath(paths.ToScoreInitial, true)),
                        new InstantCommand(() -> shooterSubsystem.setTargetVelocity(1250))
                ),
                new InstantCommand(() -> gateSubsystem.open()),
                new WaitUntilCommand(() -> shooterSubsystem.atTargetVelocity()),

                new RunIntakeCMD(intakeSubsystem, 1.0),
                new WaitCommand(2000),

                new InstantCommand(() -> intakeSubsystem.stop()),
                new InstantCommand(() -> shooterSubsystem.stop()),
                new InstantCommand(() -> gateSubsystem.close()),

                // 🟦 2️⃣ Grab first set of balls
                new InstantCommand(() -> panelsTelemetry.debug("Auto Step", "Grabbing first set of balls")),
                new ParallelCommandGroup(
                        new InstantCommand(() -> follower.followPath(paths.toAlignClose, true)),
                        new InstantCommand(() -> gateSubsystem.close())
                ),
                new ParallelCommandGroup(
                        new InstantCommand(() -> follower.followPath(paths.toGrabClose, true)),
                        new RunIntakeCMD(intakeSubsystem, 0.7)
                ),
                new WaitCommand(1000),
                new InstantCommand(() -> intakeSubsystem.stop()),

                // 🟦 3️⃣ Shoot first grabbed balls
                new InstantCommand(() -> panelsTelemetry.debug("Auto Step", "Shooting first grabbed balls")),
                new ParallelCommandGroup(
                        new InstantCommand(() -> follower.followPath(paths.toScoreClose, true)),
                        new InstantCommand(() -> shooterSubsystem.setTargetVelocity(1250))
                ),
                new InstantCommand(() -> gateSubsystem.open()),
                new WaitUntilCommand(() -> shooterSubsystem.atTargetVelocity()),

                new RunIntakeCMD(intakeSubsystem, 1.0),
                new WaitCommand(2000),

                new InstantCommand(() -> intakeSubsystem.stop()),
                new InstantCommand(() -> shooterSubsystem.stop()),
                new InstantCommand(() -> gateSubsystem.close()),

                // 🟦 4️⃣ Grab secondary balls
                new InstantCommand(() -> panelsTelemetry.debug("Auto Step", "Grabbing secondary balls")),
                new ParallelCommandGroup(
                        new InstantCommand(() -> follower.followPath(paths.toAlignSecondary, true)),
                        new InstantCommand(() -> gateSubsystem.close())
                ),
                new ParallelCommandGroup(
                        new InstantCommand(() -> follower.followPath(paths.toGrabSecondary, true)),
                        new RunIntakeCMD(intakeSubsystem, 0.7)
                ),
                new WaitCommand(1000),
                new InstantCommand(() -> intakeSubsystem.stop()),

                // 🟦 5️⃣ Shoot secondary balls
                new InstantCommand(() -> panelsTelemetry.debug("Auto Step", "Shooting secondary balls")),
                new ParallelCommandGroup(
                        new InstantCommand(() -> follower.followPath(paths.toScoreSecondary, true)),
                        new InstantCommand(() -> shooterSubsystem.setTargetVelocity(1250))
                ),
                new InstantCommand(() -> gateSubsystem.open()),
                new WaitUntilCommand(() -> shooterSubsystem.atTargetVelocity()),

                new RunIntakeCMD(intakeSubsystem, 1.0),
                new WaitCommand(2000),

                new InstantCommand(() -> intakeSubsystem.stop()),
                new InstantCommand(() -> shooterSubsystem.stop()),
                new InstantCommand(() -> gateSubsystem.close()),

                // 🟦 6️⃣ Park
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

    // ✅ Paths
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
                    .addPath(new BezierLine(new Pose(22.329, 124.063), new Pose(61.805, 82.035)))
                    .setLinearHeadingInterpolation(Math.toRadians(-37), Math.toRadians(-35))
                    .build();

            toAlignClose = follower
                    .pathBuilder()
                    .addPath(new BezierLine(new Pose(61.805, 82.035), new Pose(43.009, 84.106)))
                    .setLinearHeadingInterpolation(Math.toRadians(-35), Math.toRadians(180))
                    .build();

            toGrabClose = follower
                    .pathBuilder()
                    .addPath(new BezierLine(new Pose(43.009, 84.106), new Pose(16.726, 84.265)))
                    .setTangentHeadingInterpolation()
                    .build();

            toScoreClose = follower
                    .pathBuilder()
                    .addPath(new BezierLine(new Pose(16.726, 84.265), new Pose(62.442, 82.035)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(-35))
                    .build();

            toAlignSecondary = follower
                    .pathBuilder()
                    .addPath(new BezierLine(new Pose(62.442, 82.035), new Pose(49.899, 59.810)))
                    .setLinearHeadingInterpolation(Math.toRadians(-35), Math.toRadians(180))
                    .build();

            toGrabSecondary = follower
                    .pathBuilder()
                    .addPath(new BezierLine(new Pose(49.899, 59.810), new Pose(19.593, 59.257)))
                    .setTangentHeadingInterpolation()
                    .build();

            toScoreSecondary = follower
                    .pathBuilder()
                    .addPath(new BezierLine(new Pose(19.593, 59.257), new Pose(61.646, 82.195)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(-35))
                    .build();

            toPark = follower
                    .pathBuilder()
                    .addPath(new BezierLine(new Pose(61.646, 82.195), new Pose(53.204, 59.257)))
                    .setLinearHeadingInterpolation(Math.toRadians(-35), Math.toRadians(180))
                    .build();
        }
    }
}
