package frc.robot.commands;

import static edu.wpi.first.wpilibj2.command.Commands.waitUntil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.hopper.HopperSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.vision.AutoAimUtil;
import frc.robot.subsystems.vision.VisionSubsystem;

import java.util.concurrent.atomic.AtomicBoolean;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

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

  public static Command visionShoot(
      VisionSubsystem vision,
      Drive drive,
      ShooterSubsystem shooterSubsystem,
      HopperSubsystem hopperSubsystem) {
    return Commands.sequence(
            Commands.runOnce(
                    () -> {
                      charged = false;
                      shooterSubsystem.setStoredDistance(vision.getHubDistanceMeter((Supplier<Pose2d>) drive.poseEstimator.getEstimatedPosition()));
                    }),

            // Runs the flywheel until the target velocity is reached
            waitUntil(shooterSubsystem::targetReached)
                .andThen(Commands.waitSeconds(0.80))
                .andThen(
                    Commands.parallel(
                        startFeeding(shooterSubsystem, hopperSubsystem), Commands.runOnce(shooterSubsystem::charge))
                )
                        .andThen(Commands.waitSeconds(0.02))
                        .andThen(shooterSubsystem::calculateVelocity))
        .withTimeout(2.0);
  }

  public static Command startFeeding(
      ShooterSubsystem shooterSubsystem, HopperSubsystem hopperSubsystem) {
    return Commands.parallel(
        Commands.run(shooterSubsystem::startIntermediateMotor),
        Commands.run(hopperSubsystem::runConveyorForward),
            Commands.run(() -> charged = true)
    );
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
