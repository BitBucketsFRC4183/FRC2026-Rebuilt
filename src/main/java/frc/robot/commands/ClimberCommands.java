package frc.robot.commands;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.constants.ClimberConstants;
import frc.robot.subsystems.climber.ClimberSubsystem;
import java.util.function.DoubleSupplier;

public class ClimberCommands {

  public static Command joystickClimb(ClimberSubsystem climberSubsystem, DoubleSupplier LeftY) {

    return Commands.run(
            () -> {
              double currentHeight = climberSubsystem.getClimbHeight();
              double input = LeftY.getAsDouble();
              double desiredSpeed =
                  input
                      * ((ClimberConstants.maxHeight - currentHeight)
                          * ClimberConstants.speedConstant);
              SimpleMotorFeedforward climbFeedForward =
                  new SimpleMotorFeedforward(
                      ClimberConstants.ARM_kS,
                      ClimberConstants.ARM_kV,
                      ClimberConstants.ARM_kA,
                      0.2);
              double scale = climbFeedForward.calculate(desiredSpeed);

              if (currentHeight <= ClimberConstants.minHeight && input < 0) {
                climberSubsystem.setVoltageSupplied(0);
              }
              if (currentHeight >= ClimberConstants.maxHeight && input > 0) {
                climberSubsystem.setVoltageSupplied(0);
              }

              double voltageSupplied = input * scale;
              climberSubsystem.setVoltageSupplied(voltageSupplied);
            },
            climberSubsystem)
        .finallyDo(() -> climberSubsystem.setVoltageSupplied(0));
  }

  public static Command climberToLevelOne(ClimberSubsystem climberSubsystem) {

    return Commands.run(
            () -> {
              double currentHeight = climberSubsystem.getClimbHeight();
              double desiredSpeed =
                  (ClimberConstants.rung1Position - climberSubsystem.getClimbHeight())
                      * ClimberConstants.speedConstant;
              SimpleMotorFeedforward climbFeedForward =
                  new SimpleMotorFeedforward(
                      ClimberConstants.ARM_kS,
                      ClimberConstants.ARM_kV,
                      ClimberConstants.ARM_kA,
                      0.2);
              double scale = climbFeedForward.calculate(desiredSpeed);

              if (currentHeight <= ClimberConstants.minHeight) {
                climberSubsystem.setVoltageSupplied(0);
              }
              if (currentHeight >= ClimberConstants.maxHeight) {
                climberSubsystem.setVoltageSupplied(0);
              }

              climberSubsystem.setVoltageSupplied(scale);
            },
            climberSubsystem)
        .finallyDo(() -> climberSubsystem.setVoltageSupplied(0));
  }

  public static Command climberToGround(ClimberSubsystem climberSubsystem) {

    return Commands.run(
            () -> {
              double currentHeight = climberSubsystem.getClimbHeight();
              double desiredSpeed =
                  (-climberSubsystem.getClimbHeight()) * ClimberConstants.speedConstant;
              SimpleMotorFeedforward climbFeedForward =
                  new SimpleMotorFeedforward(
                      ClimberConstants.ARM_kS,
                      ClimberConstants.ARM_kV,
                      ClimberConstants.ARM_kA,
                      0.2);
              double scale = climbFeedForward.calculate(desiredSpeed);

              if (currentHeight <= ClimberConstants.minHeight) {
                climberSubsystem.setVoltageSupplied(0);
              }
              if (currentHeight >= ClimberConstants.maxHeight) {
                climberSubsystem.setVoltageSupplied(0);
              }

              climberSubsystem.setVoltageSupplied(scale);
            },
            climberSubsystem)
        .finallyDo(() -> climberSubsystem.setVoltageSupplied(0));
  }
}

    // IDK hopefully this hard limits the possible heights of the climber.
   /*/ if (Math.abs(input) < 0.05) {
         climberSubsystem.setVoltageSupplied(0);
       }
       if ((currentHeight <= ClimberConstants.minHeight) || (currentHeight >= ClimberConstants.maxHeight && input > 0)) {
           climberSubsystem.setVoltageSupplied(0);
         }

       double distanceToMin = currentHeight - ClimberConstants.minHeight;
       double distanceToMax = ClimberConstants.maxHeight - currentHeight;
       double closestDistance = Math.min(distanceToMin, distanceToMax);
       //the scale will scale the speeds at certain distances from the hard limits. goes 0.2 when close by and 1.0 in the middle
       double scale = MathUtil.clamp(closestDistance / ((ClimberConstants.maxHeight -1) / 2), 0.2, 1.0);


       double voltageSupplied = input * scale * maxVolts;
       return Commands.run(() -> climberSubsystem.setVoltageSupplied(voltageSupplied));

     }
   }
       */
