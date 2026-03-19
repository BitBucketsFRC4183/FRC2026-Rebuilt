package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.hopper.HopperSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;

import java.util.function.Supplier;

import static edu.wpi.first.wpilibj2.command.Commands.waitUntil;
import static frc.robot.commands.ShooterCommands.startFeeding;

public class VisionShootCommand {
    public static Command visionShoot(
            VisionSubsystem vision,
            Drive drive,
            ShooterSubsystem shooterSubsystem,
            HopperSubsystem hopperSubsystem) {
        return Commands.sequence(
                        Commands.runOnce(
                                () -> {
                                    charged = false;
                                    shooterSubsystem.setStoredDistance(
                                            vision.getHubDistanceMeter(
                                                    (Supplier<Pose2d>) drive.poseEstimator.getEstimatedPosition()));
                                }),

                        // Runs the flywheel until the target velocity is reached
                        waitUntil(shooterSubsystem::targetReached)
                                .andThen(Commands.waitSeconds(0.80))
                                .andThen(
                                        Commands.parallel(
                                                startFeeding(shooterSubsystem, hopperSubsystem),
                                                Commands.runOnce(shooterSubsystem::charge)))
                                .andThen(Commands.waitSeconds(0.02))
                                .andThen(shooterSubsystem::calculateVelocity))
                .withTimeout(2.0);
    }
}
