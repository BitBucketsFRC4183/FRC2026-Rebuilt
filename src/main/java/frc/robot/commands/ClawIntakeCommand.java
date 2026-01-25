package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm.ClawSubsystem;

public class ClawIntakeCommand extends Command {
    public final ClawSubsystem clawSubsystem;

    public ClawIntakeCommand(ClawSubsystem clawSubsystem) {
        this.clawSubsystem = clawSubsystem;
        addRequirements(clawSubsystem);
    }

    @Override
    public void execute() {
        clawSubsystem.setCentralToVoltage(7.0);
        clawSubsystem.setGrippersToVoltage(-5);
    }




    public void end(boolean interrupted) {
        if (interrupted) {
            clawSubsystem.setCentralToVoltage(0);
            clawSubsystem.setGrippersToVoltage(0);
        }


    }
}
