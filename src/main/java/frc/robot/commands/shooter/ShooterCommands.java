package frc.robot.commands.shooter;

import static edu.wpi.first.wpilibj2.command.Commands.waitUntil;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.hopper.HopperSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class ShooterCommands {
  private static boolean charged = false;

  public static Command shootAtRPS(
      double targetVelocity, ShooterSubsystem shooterSubsystem, HopperSubsystem hopperSubsystem) {
    return Commands.sequence(
        Commands.runOnce(() -> shooterSubsystem.setTargetVelocity(targetVelocity)),
        waitUntil(shooterSubsystem::targetReached)
            .andThen(Commands.waitSeconds(0.80))
            .andThen(startFeeding(shooterSubsystem, hopperSubsystem)));
  }

  public static Command startFeeding(
      ShooterSubsystem shooterSubsystem, HopperSubsystem hopperSubsystem) {
    return Commands.parallel(
        Commands.run(shooterSubsystem::startIntermediateMotor),
        Commands.run(hopperSubsystem::runConveyorForward),
        Commands.run(() -> charged = true));
  }

  public static Command reset(ShooterSubsystem shooterSubsystem, HopperSubsystem hopperSubsystem) {
    return Commands.parallel(
        Commands.runOnce(shooterSubsystem::resetStoredDistance),
        Commands.runOnce(shooterSubsystem::stopFlywheel),
        stopFeeding(shooterSubsystem, hopperSubsystem));
  }

  public static Command stopFeeding(
      ShooterSubsystem shooterSubsystem, HopperSubsystem hopperSubsystem) {
    return Commands.parallel(
        Commands.runOnce(shooterSubsystem::stopIntermediateMotor),
        Commands.runOnce(hopperSubsystem::stopConveyor));
  }

  public static boolean isCharged() {
    return charged;
  }

  public static void setCharged(boolean charged) {
    ShooterCommands.charged = charged;
  }
}
