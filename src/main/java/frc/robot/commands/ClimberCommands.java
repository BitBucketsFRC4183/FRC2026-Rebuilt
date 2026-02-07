package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.climber.ClimberSubsystem;
import java.util.function.DoubleSupplier;

public class ClimberCommands {
  public static Command joystickClimb(ClimberSubsystem climberSubsystem, DoubleSupplier LeftY) {
    double voltageSupplied = LeftY.getAsDouble() * 5;
    System.out.println("test");
    return Commands.run(() -> climberSubsystem.setVoltageSupplied(voltageSupplied));


  }
  public static Command climberToGround(ClimberSubsystem climberSubsystem){
    return Commands.run(climberSubsystem::moveClimbToGround);
  }
  public static Command climberToLevel1(ClimberSubsystem climberSubsystem){
    return  Commands.run(climberSubsystem::moveClimbToLevel1);
  }

}
