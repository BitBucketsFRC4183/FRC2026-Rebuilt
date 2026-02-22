package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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
    return Commands.runOnce(intake::hold, intake).withName("Intake.Hold");
  }

  // Stops intake and retracts forebar
  public static Command stopAndStow(IntakeSubsystem intake) {
    return Commands.sequence(stow(intake)).withName("Intake.StopAndStow");
  }

  // Instant deploy and intake
  public static Command deployAndIntake(IntakeSubsystem intake) {
    return Commands.sequence(deploy(intake), intake(intake)).withName("Intake.DeployAndIntake");
  }

  // Intake until Current Threshold reached
  //  public static Command intakeUntilCurrent(IntakeSubsystem intake, double currentThreshold) {
  //
  //    return Commands.startEnd(intake::intake, intake::hold, intake)
  //        .until(() -> intake.getMotorCurrent() >= currentThreshold)
  //        .withName("Intake.IntakeUntilCurrent");
  //  }
}
