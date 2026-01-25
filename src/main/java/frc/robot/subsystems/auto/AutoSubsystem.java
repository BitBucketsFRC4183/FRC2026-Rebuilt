package frc.robot.subsystems.auto;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.RobotConfig;
// import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
// import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.DriveSubsystem;

public class AutoSubsystem extends SubsystemBase {

  private final DriveSubsystem drive;

  public AutoSubsystem(DriveSubsystem drive) {
    this.drive = drive;

    configureAutoBuilder();
  }

  private void configureAutoBuilder() {
    try {
      RobotConfig config = RobotConfig.fromGUISettings();

      //     AutoBuilder.configureHolonomic(
      //        drive::getPose,
      //         drive::resetPose,
      //        drive::getChassisSpeeds,
      //        drive::drive,
      //        new HolonomicPathFollowerConfig(
      //            config.getMaxLinearVelocity(), config.getDriveBaseRadius(), new
      // ReplanningConfig()),
      //       () ->
      //           DriverStation.getAlliance()
      //               .map(alliance -> alliance == DriverStation.Alliance.Red)
      //               .orElse(false),
      //       drive);
    } catch (Exception e) {
      DriverStation.reportError("Failed to configure AutoBuilder", e.getStackTrace());
    }
  }

  /** Stops drivetrain */
  // public Command stop() {
  //  return drive.run(() -> drive.drive(new edu.wpi.first.math.kinematics.ChassisSpeeds()));
  // }

  /** Example Choreo-based routine */
  public Command midShootAuto() {
    return new PathPlannerAuto("MidShootExample");
  }
}
