package frc.robot.commands;

import com.ctre.phoenix6.configs.FeedbackConfigs;
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

  public static Command increaseClimberLengthLevelOne(ClimberSubsystem climberSubsystem) {

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
        .until(
            () ->
                Math.abs(ClimberConstants.rung1Position - climberSubsystem.getClimbHeight()) < 0.01)
        .finallyDo(
            () -> {
              climberSubsystem.setVoltageSupplied(0);
            });
  }
    public static Command increaseClimberLengthLevelTwo(ClimberSubsystem climberSubsystem) {

        return Commands.run(
                        () -> {
                            double currentHeight = climberSubsystem.getClimbHeight();
                            double desiredSpeed =
                                    (ClimberConstants.rung2Position - climberSubsystem.getClimbHeight())
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
                .until(
                        () ->
                                Math.abs(ClimberConstants.rung2Position - climberSubsystem.getClimbHeight()) < 0.01)
                .finallyDo(
                        () -> {
                            climberSubsystem.setVoltageSupplied(0);
                        });
    }

  public static Command decreaseClimberLength(ClimberSubsystem climberSubsystem) {

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
        .until(() -> Math.abs(-climberSubsystem.getClimbHeight()) < 0.01)
        .finallyDo(() -> climberSubsystem.setVoltageSupplied(0));
  }

  public static Command climberServoUp(ClimberSubsystem climberSubsystem) {
    return Commands.runOnce(
        () -> {
          double servoPosition = 1.0;
          climberSubsystem.setClimbServoPosition(servoPosition);
          Commands.waitSeconds(2);
        });
  }

  public static Command climberServoDown(ClimberSubsystem climberSubsystem) {
    return Commands.runOnce(
        () -> {
          double servoPosition = 0;
          climberSubsystem.setClimbServoPosition(servoPosition);
            Commands.waitSeconds(2);
        });
  }

  public static Command baseServoUp(ClimberSubsystem climberSubsystem) {
    return Commands.runOnce(
        () -> {
          double servoPosition = 1.0;
          climberSubsystem.setBaseServoPosition(servoPosition);
            Commands.waitSeconds(2);
        });
  }

  public static Command baseServoDown(ClimberSubsystem climberSubsystem) {
    return Commands.runOnce(
        () -> {
          double servoPosition = 0;
          climberSubsystem.setBaseServoPosition(servoPosition);
            Commands.waitSeconds(2);
        });
  }

  public static Command climbToGround(ClimberSubsystem climberSubsystem) {
    return ClimberCommands.baseServoDown(climberSubsystem)
        .andThen(
            ClimberCommands.increaseClimberLengthLevelOne(climberSubsystem)
                .andThen(ClimberCommands.climberServoDown(climberSubsystem)));
  }

  public static Command climbToLevelOne(ClimberSubsystem climberSubsystem) {
    return ClimberCommands.increaseClimberLengthLevelOne(climberSubsystem)
        .andThen(
            ClimberCommands.climberServoUp(climberSubsystem)
                .andThen(
                    ClimberCommands.decreaseClimberLength(climberSubsystem)
                        .andThen(ClimberCommands.baseServoUp(climberSubsystem))));
  }
  ;

  public static Command climbToLevelTwo(ClimberSubsystem climberSubsystem) {
    return ClimberCommands.climberServoDown(climberSubsystem)
        .andThen(
            ClimberCommands.increaseClimberLengthLevelTwo(climberSubsystem)
                .andThen(
                    ClimberCommands.climberServoUp(climberSubsystem)
                        .andThen(
                            ClimberCommands.baseServoDown(climberSubsystem)
                                .andThen(
                                    ClimberCommands.decreaseClimberLength(climberSubsystem)
                                        .andThen(ClimberCommands.baseServoUp(climberSubsystem))))));
  }

  public static Command climbToLevelThree(ClimberSubsystem climberSubsystem) {
    return ClimberCommands.climberServoDown(climberSubsystem)
        .andThen(
            ClimberCommands.increaseClimberLengthLevelTwo(climberSubsystem)
                .andThen(
                    ClimberCommands.climberServoUp(climberSubsystem)
                        .andThen(
                            ClimberCommands.baseServoDown(climberSubsystem)
                                .andThen(
                                    ClimberCommands.decreaseClimberLength(climberSubsystem)
                                        .andThen(ClimberCommands.baseServoUp(climberSubsystem))))));
  }
}
