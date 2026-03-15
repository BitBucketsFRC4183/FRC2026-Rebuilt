package frc.robot.commands;

import static edu.wpi.first.wpilibj2.command.Commands.waitUntil;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.hopper.HopperSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import java.util.function.DoubleSupplier;

public class ShooterCommands {
  public static Command shootAtRPS(
      double targetVelocity, ShooterSubsystem shooterSubsystem, HopperSubsystem hopperSubsystem) {
    return Commands.sequence(
        Commands.runOnce(() -> shooterSubsystem.setTargetVelocity(targetVelocity)),
        waitUntil(shooterSubsystem::targetReached)
            .andThen(Commands.waitSeconds(0.80))
            .andThen(startFeeding(shooterSubsystem, hopperSubsystem)));
  }

  public static Command visionShoot(
      DoubleSupplier distanceSupplier,
      ShooterSubsystem shooterSubsystem,
      HopperSubsystem hopperSubsystem) {

    return Commands.sequence(
            Commands.runOnce(
                    () -> {
                      shooterSubsystem.setStoredDistance(distanceSupplier.getAsDouble());
                    }),

                // Runs the flywheel until the target velocity is reached
                waitUntil(shooterSubsystem::targetReached)
                .andThen(Commands.waitSeconds(0.80))
                .andThen(
                    Commands.parallel(
                        startFeeding(shooterSubsystem, hopperSubsystem))))
        .withTimeout(0.5);
  }

  public static Command startFeeding(
      ShooterSubsystem shooterSubsystem, HopperSubsystem hopperSubsystem) {
    return Commands.parallel(
        Commands.run(shooterSubsystem::startIntermediateMotor),
        Commands.run(hopperSubsystem::runConveyorForward));
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
}
