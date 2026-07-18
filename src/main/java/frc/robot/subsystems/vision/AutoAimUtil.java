package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.constants.AprilTagLabel;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class AutoAimUtil {
  /// fancy notation
  /// function that give you hub pose based on alliance color
  public static Pose3d getTargetHubPose3d() {
    if (DriverStation.getAlliance().isPresent()) {
      if (DriverStation.getAlliance().get().equals(Alliance.Red)) {
        return AprilTagLabel.RedHubPose3d;
      } else {
        return AprilTagLabel.BlueHubPose3d;
      }
    } else {
      return AprilTagLabel.BlueHubPose3d;
    }
  }

  // For Logging
  public static Pose2d getTargetHubPose2d() {
    var p = getTargetHubPose3d().toPose2d();
    return getTargetHubPose3d().toPose2d();
  }

  // dist to hub meters
  public static double getDistanceToHub(Supplier<Pose2d> robotPose) {
    boolean weInNeutralZone = false;
    //    if (robotPose.get().getX() > VisionConstants.MidGameMin
    //        && robotPose.get().getX() < VisionConstants.MidGameMax) {
    //      weInNeutralZone = true;
    //      return 0;
    //    }
    Logger.recordOutput("Aim/weInNeutralZone", weInNeutralZone);

    var d = robotPose.get().getTranslation().getDistance(getTargetHubPose2d().getTranslation());

    return d;
  }

  // pointed angle to hub, where 0rads is where the balls exit, therefore adding 180 is not
  // neccessary
  public static Rotation2d getAngleToHub(Supplier<Pose2d> robotPose) {

    double flip;
    if (DriverStation.getAlliance().isPresent()) {
      if (DriverStation.getAlliance().get().equals(Alliance.Red)) {
        flip = Math.PI;
      } else {
        flip = 0;
      }
    } else {
      flip = 0;
    }

    double x_diff = robotPose.get().getX() - getTargetHubPose2d().getX();
    double y_diff = robotPose.get().getY() - getTargetHubPose2d().getY();
    double theta = Math.atan2(y_diff, x_diff);

    var r = Rotation2d.fromRadians(theta + flip);

    //    Logger.recordOutput("Aim/angleToHub", r);
    return r;
  }
}
