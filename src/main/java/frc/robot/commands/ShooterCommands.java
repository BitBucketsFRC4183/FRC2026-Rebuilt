package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class ShooterCommands {
    public static Command storeDistance(ShooterSubsystem shooterSubsystem, double distance) {
        return Commands.runOnce(
                () -> {
                    shooterSubsystem.setStoredDistance(distance);
                    shooterSubsystem.calculateVelocity();
                });
    }
    public static Command revFlywheels(ShooterSubsystem shooterSubsystem) {
        return Commands.sequence(
                // Waits for the distance from vision
                Commands.waitUntil(shooterSubsystem::distanceStored),
                Commands.run(() -> System.out.println(ShooterSubsystem.getTargetVelocity())),
                Commands.run(shooterSubsystem::setTargetVelocity, shooterSubsystem)
                        .until(shooterSubsystem::targetReached)
                        .andThen(
                                Commands.parallel(
                                        Commands.run(shooterSubsystem::setTargetVelocity),
                                        Commands.run(shooterSubsystem::startIntermediateMotors))));
    }
    public static Command reset(ShooterSubsystem shooterSubsystem) {
        return Commands.parallel(
                Commands.runOnce(shooterSubsystem::resetStoredDistance),
                Commands.runOnce(shooterSubsystem::stop)
        );
    }
}
