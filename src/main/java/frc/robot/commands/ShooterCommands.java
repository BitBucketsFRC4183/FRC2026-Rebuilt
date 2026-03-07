package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.hopper.HopperSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;

import static edu.wpi.first.wpilibj2.command.Commands.waitUntil;

public class ShooterCommands {
  public static Command storeDistance(ShooterSubsystem shooterSubsystem, double distance) {
    return Commands.none();
  }

  public static Command shootAtRPS(
      double targetVelocity, ShooterSubsystem shooterSubsystem, HopperSubsystem hopperSubsystem) {
    return Commands.sequence(
        Commands.runOnce(() -> shooterSubsystem.setTargetVelocity(targetVelocity)),
        waitUntil(shooterSubsystem::targetReached).andThen(Commands.waitSeconds(0.5))
            .andThen(feed(shooterSubsystem, hopperSubsystem)));
  }

  public static Command visionShoot(
      double distance, ShooterSubsystem shooterSubsystem, HopperSubsystem hopperSubsystem) {
    return Commands.sequence(
        Commands.runOnce(
                () -> {
                  shooterSubsystem.setStoredDistance(distance);
                  shooterSubsystem.calculateVelocity();
                })
            // Runs the flywheel until the controller is released
            .until(shooterSubsystem::targetReached)
            .andThen(feed(shooterSubsystem, hopperSubsystem)));
  }

  public static Command feed(ShooterSubsystem shooterSubsystem, HopperSubsystem hopperSubsystem) {
    return Commands.parallel(
        Commands.run(shooterSubsystem::startFeeding),
        Commands.run(hopperSubsystem::runConveyorForward));
  }

  public static Command reset(ShooterSubsystem shooterSubsystem, HopperSubsystem hopperSubsystem) {
    return Commands.parallel(
        Commands.runOnce(shooterSubsystem::resetStoredDistance),
        Commands.runOnce(shooterSubsystem::stop),
        Commands.runOnce(hopperSubsystem::stopConveyor));
  }
}
