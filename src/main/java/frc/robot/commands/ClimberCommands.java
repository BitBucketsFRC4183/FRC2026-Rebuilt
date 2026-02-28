package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.constants.ClimberConstants;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.drive.Drive;

import java.util.function.DoubleSupplier;

public class ClimberCommands {

  public static Command joystickClimb(ClimberSubsystem climberSubsystem, DoubleSupplier LeftY) {

    return Commands.run(
            () -> {
              double currentHeight = climberSubsystem.getClimbHeight();
              double input = LeftY.getAsDouble();

              if (Math.abs(input) < 0.05) {
                climberSubsystem.setVoltageSupplied(0);
                return;
              }

              double desiredSpeed =
                  input
                      * (ClimberConstants.maxHeight - currentHeight)
                      * ClimberConstants.speedConstant;

              climberSubsystem.setTargetHeight(desiredSpeed);
            },
            climberSubsystem)
        .finallyDo(() -> climberSubsystem.setVoltageSupplied(0));
  }

  public static Command increaseClimberLengthLevelOne(ClimberSubsystem climberSubsystem) {

    return Commands.run(
            () -> {
              double currentHeight = climberSubsystem.getClimbHeight();
              climberSubsystem.setTargetHeight(ClimberConstants.rung1Position);

              if (currentHeight <= ClimberConstants.minHeight) {
                climberSubsystem.setVoltageSupplied(0);
              }
              if (currentHeight >= ClimberConstants.maxHeight) {
                climberSubsystem.setVoltageSupplied(0);
              }
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
              climberSubsystem.setTargetHeight(ClimberConstants.rung2Position);

              if (currentHeight <= ClimberConstants.minHeight) {
                climberSubsystem.setVoltageSupplied(0);
              }
              if (currentHeight >= ClimberConstants.maxHeight) {
                climberSubsystem.setVoltageSupplied(0);
              }
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
              climberSubsystem.setTargetHeight(-currentHeight);

              if (currentHeight <= ClimberConstants.minHeight) {
                climberSubsystem.setVoltageSupplied(0);
              }
              if (currentHeight >= ClimberConstants.maxHeight) {
                climberSubsystem.setVoltageSupplied(0);
              }
            },
            climberSubsystem)
        .until(() -> Math.abs(-climberSubsystem.getClimbHeight()) < 0.5)
        .finallyDo(() -> climberSubsystem.setVoltageSupplied(0));
  }

  public static Command climberServoUp(ClimberSubsystem climberSubsystem) {
    return Commands.runOnce(
            () -> {
              double servoPosition = 1.0;
              climberSubsystem.setClimbServoPosition(servoPosition);
              climberSubsystem.setkG(ClimberConstants.ARM_kGUp);
            })
        .andThen(() -> Commands.waitSeconds(2));
  }

  public static Command climberServoDown(ClimberSubsystem climberSubsystem) {
    return Commands.runOnce(
            () -> {
              double servoPosition = 0;
              climberSubsystem.setClimbServoPosition(servoPosition);
              climberSubsystem.setkG(ClimberConstants.ARM_kGDown);
            })
        .andThen(() -> Commands.waitSeconds(2));
  }

  public static Command baseServoUp(ClimberSubsystem climberSubsystem) {
    return Commands.runOnce(
            () -> {
              double servoPosition = 1.0;
              climberSubsystem.setBaseServoPosition(servoPosition);
            })
        .andThen(() -> Commands.waitSeconds(2));
  }

  public static Command baseServoDown(ClimberSubsystem climberSubsystem) {
    return Commands.runOnce(
            () -> {
              double servoPosition = 0;
              climberSubsystem.setBaseServoPosition(servoPosition);
            })
        .andThen(() -> Commands.waitSeconds(2));
  }

  public static Command climbToGround(ClimberSubsystem climberSubsystem) {
    return ClimberCommands.baseServoDown(climberSubsystem)
        .andThen(
            ClimberCommands.increaseClimberLengthLevelOne(climberSubsystem)
                .andThen(ClimberCommands.climberServoDown(climberSubsystem))
                .andThen(ClimberCommands.decreaseClimberLength(climberSubsystem)));
  }

  public static Command climbToLevelOne(ClimberSubsystem climberSubsystem, Drive drive) {
    return ClimberCommands.increaseClimberLengthLevelOne(climberSubsystem)
        .andThen(Commands.deadline(Commands.waitSeconds(3), new RobotRelativeDriveCommand(drive, ()-> 0.2, () ->0, () -> 0))
                .andThen(
                    ClimberCommands.decreaseClimberLength(climberSubsystem)
                        .andThen()));
  }
  ;

  public static Command climbToLevelTwoAndThree(ClimberSubsystem climberSubsystem) {
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

  public static Command climbZeroing(ClimberSubsystem climberSubsystem) {

    return Commands.run(() -> climberSubsystem.setTargetHeight(ClimberConstants.maxHeight)).until(() -> climberSubsystem.getClimbHeight() == ClimberConstants.maxHeight - 0.5)
            .andThen(Commands.runOnce(climberSubsystem::resetPosition)
                    .andThen(Commands.runOnce(climberSubsystem::setInverted)));

  }
}
