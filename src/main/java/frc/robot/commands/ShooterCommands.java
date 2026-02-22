package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.hopper.HopperSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class ShooterCommands {
  public static Command storeDistance(ShooterSubsystem shooterSubsystem, double distance) {
    return Commands.runOnce(
        () -> {
          shooterSubsystem.setStoredDistance(distance);
          shooterSubsystem.calculateVelocity();
        });
  }

  public static Command revFlywheels(
      ShooterSubsystem shooterSubsystem, HopperSubsystem hopperSubsystem) {
    return Commands.sequence(
        // Waits for the distance from vision
        Commands.waitUntil(shooterSubsystem::distanceStored),
        // Commands.run(() -> System.out.println(ShooterSubsystem.getTargetVelocity())),
        // Runs the flywheel until the controller is released
        Commands.run(shooterSubsystem::setTargetVelocity, shooterSubsystem)
            .until(shooterSubsystem::targetReached)
            .andThen(
                Commands.parallel(
                    Commands.run(shooterSubsystem::setTargetVelocity),
                    Commands.run(shooterSubsystem::startFeeding),
                    Commands.run(hopperSubsystem::runConveyorForward))));
  }

  public static Command reset(ShooterSubsystem shooterSubsystem, HopperSubsystem hopperSubsystem) {
    return Commands.parallel(
        Commands.runOnce(shooterSubsystem::resetStoredDistance),
        Commands.runOnce(shooterSubsystem::stop),
        Commands.runOnce(hopperSubsystem::stopConveyor));
  }
}
