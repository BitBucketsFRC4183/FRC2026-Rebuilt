package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

public class ClawOuttakeCommand extends Command {
    private final ClawSubsystem clawSubsystem;


    public ClawOuttakeCommand(ClawSubsystem clawSubsystem) {
        this.clawSubsystem = clawSubsystem;
        addRequirements(clawSubsystem);
    }

    public void execute() {
        clawSubsystem.setCentralToVoltage(7.0);
        clawSubsystem.setGrippersToVoltage(2);
    }

    public void end(boolean interrupted) {
        if (interrupted) {
            clawSubsystem.setCentralToVoltage(0);
            clawSubsystem.setGrippersToVoltage(0);
        }
    }
}
