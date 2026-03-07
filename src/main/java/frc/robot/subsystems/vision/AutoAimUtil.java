package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.*;
import frc.robot.constants.AprilTagLabel;

public class AutoAimUtil {
  /// fancy notation
  /// function that give you hub pose based on alliance color
  public static Pose3d getTargetHubPose3d() {
    return AprilTagLabel.BlueHubPose3d;
  }

  // For Logging
  public static Pose2d getTargetHubPose2d() {
    var p = getTargetHubPose3d().toPose2d();
    return getTargetHubPose3d().toPose2d();
  }

  // dist to hub meters
  public static double getDistanceToHub(Pose2d robotPose) {
    var d = robotPose.getTranslation().getDistance(getTargetHubPose2d().getTranslation());
    return d;
  }

  // pointed angle to hub, where 0rads is where the balls exit, therefore adding 180 is not
  // neccessary
  public static Rotation2d getAngletoHub(Pose2d robotPose) {
    double x_diff = robotPose.getX() - getTargetHubPose2d().getX();
    double y_diff = robotPose.getY() - getTargetHubPose2d().getY();
    double theta = Math.atan2(y_diff, x_diff);

    var r = Rotation2d.fromRadians(theta);
    return r;
  }
}
