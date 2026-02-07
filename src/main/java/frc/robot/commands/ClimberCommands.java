package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.climber.ClimberSubsystem;

public class ClimberCommands {
  public static Command climb(ClimberSubsystem climberSubsystem, double LeftY) {
    double voltageSupplied = LeftY * 12;
    climberSubsystem.setVoltageSupplied(voltageSupplied);
    return Commands.run(climberSubsystem::moveClimbToLevel1, climberSubsystem);
  }
}
