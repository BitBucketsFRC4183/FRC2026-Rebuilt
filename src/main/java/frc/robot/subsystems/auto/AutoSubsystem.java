package frc.robot.subsystems.auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.subsystems.drive.DriveSubsystem;

public class AutoSubsystem extends SubsystemBase {

  private final DriveSubsystem drive;

  public AutoSubsystem(DriveSubsystem drive) {
    this.drive = drive;

    configureAutoBuilder();
    registerNamedCommands();
  }

  //Setup PathPlanner

  private void configureAutoBuilder() {
    AutoBuilder.configureHolonomic(
            drive::getPose,
            drive::resetPose,
            drive::getRobotRelativeSpeeds,
            drive::driveRobotRelative,
            new PPHolonomicDriveController(
                    new PIDConstants(5.0, 0.0, 0.0),
                    new PIDConstants(5.0, 0.0, 0.0)
            ),
            RobotConfig.fromGUISettings(),
            drive::shouldFlipPath,
            drive
    );
  }

  //Named Commands

  private void registerNamedCommands() {

    NamedCommands.registerCommand(
            "Stop",
            stop()
    );

    NamedCommands.registerCommand(
            "Shoot",
            shoot()
    );

    NamedCommands.registerCommand(
            "Climb",
            climb()
    );
  }

  // Auto Actions

  public Command stop() {
    return Commands.runOnce(drive::stop, drive);
  }

  public Command shoot() {
    return Commands.print("Shooting!");
    // replace with real shooter command
  }

  public Command climb() {
    return Commands.print("Climbing!");
    // replace with real climb command
  }
}
