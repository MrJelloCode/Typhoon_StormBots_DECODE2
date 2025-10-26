package org.firstinspires.ftc.teamcode.pedroPathing.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems.GateSubsystem;

/**
 * Runs the intake at a given power while the gate is open.
 * Automatically stops once the gate closes.
 */
public class RunIntakeCMD extends CommandBase {

    private final IntakeSubsystem intake;
    private final GateSubsystem gate;
    private final double power;

    public RunIntakeCMD(IntakeSubsystem intake, GateSubsystem gate, double power) {
        this.intake = intake;
        this.gate = gate;
        this.power = power;
        // no addRequirements() because IntakeSubsystem is not a Subsystem
    }

    @Override
    public void initialize() {
        intake.setPower(power);
    }

    @Override
    public void end(boolean interrupted) {
        intake.stop();
    }

    @Override
    public boolean isFinished() {
        // End automatically when the gate closes
        return !gate.isOpen();
    }
}
