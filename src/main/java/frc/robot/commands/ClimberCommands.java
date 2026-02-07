package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.constants.ClimberConstants;
import frc.robot.subsystems.climber.ClimberSubsystem;
import java.util.function.DoubleSupplier;

public class ClimberCommands {

  public static Command joystickClimb(ClimberSubsystem climberSubsystem, DoubleSupplier LeftY) {
    double maxVolts = 12;
    double currentHeight = climberSubsystem.getClimbHeight();
    double input = LeftY.getAsDouble();
    System.out.println("test");

    if (Math.abs(input) < 0.05) {
      climberSubsystem.setVoltageSupplied(0);
      if ((currentHeight <= 0 && input < 0) || (currentHeight >= ClimberConstants.maxHeight && input > 0)) {
        climberSubsystem.setVoltageSupplied(0);
      }
    }

    double distanceToMin = currentHeight - ClimberConstants.minHeight;
    double distanceToMax = ClimberConstants.maxHeight - currentHeight;
    double closestDistance = Math.min(distanceToMin, distanceToMax);
    double scale = MathUtil.clamp(closestDistance / ((ClimberConstants.maxHeight -1) / 2), 0.2, 1.0);
    double voltageSupplied = input * scale * maxVolts;

    return Commands.run(() -> climberSubsystem.setVoltageSupplied(voltageSupplied));

  }
}
