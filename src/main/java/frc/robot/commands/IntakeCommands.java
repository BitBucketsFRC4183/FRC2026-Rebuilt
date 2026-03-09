package frc.robot.commands;

import static frc.robot.constants.IntakeConstants.INTAKE_SERVO_DEPLOY_PULSEWIDTH;
import static frc.robot.constants.IntakeConstants.INTAKE_SERVO_STOW_PULSEWIDTH;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.constants.IntakeConstants;
import frc.robot.subsystems.intake.IntakeSubsystem;

/**
 * Factory class for Intake-related commands. All commands assume IntakeSubsystem owns its internal
 * state machine.
 */
public final class IntakeCommands {

  private IntakeCommands() {}

  // Stows Intake
  public static Command stow(IntakeSubsystem intake) {
    return Commands.runOnce(intake::stow, intake).withName("Intake.Stow");
  }

  // Deploys Intake without motor
  public static Command deploy(IntakeSubsystem intake) {
    return Commands.runOnce(intake::deploy, intake).withName("Intake.Deploy");
  }

  // Runs Intake in while being held out
  public static Command intake(IntakeSubsystem intake) {
    return Commands.startEnd(intake::intake, intake::hold, intake).withName("Intake.Intake");
  }

  // Runs Intake Out whilst held
  public static Command outtake(IntakeSubsystem intake) {
    return Commands.startEnd(intake::outtake, intake::hold, intake).withName("Intake.Outtake");
  }

  // holds forebar out
  public static Command hold(IntakeSubsystem intake) {
    return Commands.runOnce(intake::deploy, intake).withName("Intake.Hold");
  }

  // Stops intake and retracts forebar
  public static Command stopAndStow(IntakeSubsystem intake) {
    return Commands.sequence(stow(intake)).withName("Intake.StopAndStow");
  }

  // Instant deploy and intake
  public static Command deployAndIntake(IntakeSubsystem intake) {
    return Commands.sequence(deploy(intake), intake(intake)).withName("Intake.DeployAndIntake");
  }

  public static Command moveServoTo0(IntakeSubsystem intake) {
    return Commands.runOnce(() -> intake.io.setServoAngle(IntakeConstants.SERVO_0), intake)
        .withName("Intake.MoveServo0");
  }

  public static Command moveServoTo90(IntakeSubsystem intake) {
    return Commands.runOnce(() -> intake.io.setServoAngle(IntakeConstants.SERVO_90), intake)
        .withName("Intake.MoveServo90");
  }

  public static Command setServoPulsePositive(IntakeSubsystem intake) {
    return Commands.runOnce(
            () -> intake.io.setServoPulseWidth(INTAKE_SERVO_DEPLOY_PULSEWIDTH), intake)
        .withName("Intake.MoveServo90");
  }

  public static Command setServoPulseNegative(IntakeSubsystem intake) {
    return Commands.runOnce(
            () -> intake.io.setServoPulseWidth(INTAKE_SERVO_STOW_PULSEWIDTH), intake)
        .withName("Intake.MoveServo90");
  }
}
