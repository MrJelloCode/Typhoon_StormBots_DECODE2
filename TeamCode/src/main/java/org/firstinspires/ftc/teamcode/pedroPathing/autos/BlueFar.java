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

@Autonomous(name = "Blue Far Auto", group = "Autonomous")
@Configurable
public class BlueFar extends OpMode {

    private TelemetryManager panelsTelemetry;
    private Follower follower;
    private Paths paths;

    // Subsystems
    private IntakeSubsystem intakeSubsystem;
    private ShooterSubsystem shooterSubsystem;
    private GateSubsystem gateSubsystem;

    private CommandScheduler scheduler = CommandScheduler.getInstance();
    private int pathState = 0;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(72, 8, Math.toRadians(90)));

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
                new InstantCommand(() -> gateSubsystem.close()),
                new InstantCommand(() -> follower.followPath(paths.toScoreInitial, true)),

                new InstantCommand(() -> shooterSubsystem.setTargetVelocity(1250)),
                new WaitUntilCommand(() -> shooterSubsystem.isAtSpeed(1250, 50)),

                new InstantCommand(() -> gateSubsystem.open()),
                new InstantCommand(() -> intakeSubsystem.intakeIn()),
                new WaitCommand(1000),

                new InstantCommand(() -> intakeSubsystem.stop()),
                new InstantCommand(() -> shooterSubsystem.stop()),
                new InstantCommand(() -> gateSubsystem.close()),

                // 2️⃣ Grab close balls
                new ParallelCommandGroup(
                        new InstantCommand(() -> follower.followPath(paths.toAlignFirst, true)),
                        new InstantCommand(() -> intakeSubsystem.setPower(0.7)),
                        new InstantCommand(() -> gateSubsystem.close())
                ),
                new InstantCommand(() -> follower.followPath(paths.toGrabFirst, true)),
                new WaitCommand(300),

                // 3️⃣ Shoot close balls
                new InstantCommand(() -> intakeSubsystem.stop()),
                new InstantCommand(() -> follower.followPath(paths.toScoreFirst, true)),
                new InstantCommand(() -> shooterSubsystem.setTargetVelocity(1250)),

                new WaitUntilCommand(() -> shooterSubsystem.isAtSpeed(1250, 50)),
                new InstantCommand(() -> gateSubsystem.open()),
                new InstantCommand(() -> intakeSubsystem.intakeIn()),
                new WaitCommand(1000),

                new InstantCommand(() -> intakeSubsystem.stop()),
                new InstantCommand(() -> shooterSubsystem.stop()),
                new InstantCommand(() -> gateSubsystem.close()),

                // 4️⃣ Grab secondary balls
                new ParallelCommandGroup(
                        new InstantCommand(() -> follower.followPath(paths.toAlignSecond, true)),
                        new InstantCommand(() -> intakeSubsystem.setPower(0.7)),
                        new InstantCommand(() -> gateSubsystem.close())
                ),
                new InstantCommand(() -> follower.followPath(paths.toGrabSecond, true)),
                new WaitCommand(300),

                // 5️⃣ Shoot secondary balls
                new InstantCommand(() -> intakeSubsystem.stop()),
                new InstantCommand(() -> follower.followPath(paths.toScoreSecond, true)),
                new InstantCommand(() -> shooterSubsystem.setTargetVelocity(1250)),

                new WaitUntilCommand(() -> shooterSubsystem.isAtSpeed(1250, 50)),
                new InstantCommand(() -> gateSubsystem.open()),
                new InstantCommand(() -> intakeSubsystem.intakeIn()),
                new WaitCommand(1000),

                new InstantCommand(() -> intakeSubsystem.stop()),
                new InstantCommand(() -> shooterSubsystem.stop()),
                new InstantCommand(() -> gateSubsystem.close()),

                // 6️⃣ Park
                new InstantCommand(() -> follower.followPath(paths.toPark, true)),
                new WaitCommand(1000),

                new InstantCommand(() -> {
                    intakeSubsystem.stop();
                    shooterSubsystem.stop();
                    gateSubsystem.close();
                })
        ));
    }

    @Override
    public void loop() {
        follower.update();
        scheduler.run();

        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.update(telemetry);
    }

    // ✅ Paths for Pedro Pathing
    public static class Paths {
        public PathChain toScoreInitial;
        public PathChain toAlignFirst;
        public PathChain toGrabFirst;
        public PathChain toScoreFirst;
        public PathChain toAlignSecond;
        public PathChain toGrabSecond;
        public PathChain toScoreSecond;
        public PathChain toPark;

        public Paths(Follower follower) {
            toScoreInitial = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(54.570, 9.228), new Pose(60.152, 23.127))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(-65))
                    .build();

            toAlignFirst = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(60.152, 23.127), new Pose(49.221, 35.204))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(-65), Math.toRadians(180))
                    .build();

            toGrabFirst = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(49.221, 35.204), new Pose(10.481, 36.228))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            toScoreFirst = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(10.481, 36.228), new Pose(61.063, 23.127))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(-65))
                    .build();

            toAlignSecond = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(61.063, 23.127), new Pose(12.106, 28.832))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(-65), Math.toRadians(250))
                    .build();

            toGrabSecond = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(12.106, 28.832), new Pose(12.425, 12.265))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(250), Math.toRadians(250))
                    .build();

            toScoreSecond = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(12.425, 12.265), new Pose(60.608, 23.013))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(250), Math.toRadians(-65))
                    .build();

            toPark = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(60.608, 23.013), new Pose(56.708, 41.735))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(-65), Math.toRadians(180))
                    .build();
        }
    }
}