package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.constants.AprilTagLabel;
import frc.robot.constants.VisionConstant;

public class AutoAimCalculation {
  /// fancy notation
  /// function that give you hub pose based on alliance color
  public static Pose3d getTargetHubPose3d() {
    var alliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue);
    return (alliance == DriverStation.Alliance.Red)
        ? AprilTagLabel.RedHubPose3d
        : AprilTagLabel.BlueHubPose3d;
  }

  public static Pose2d getTargetHubPose2d() {
    return getTargetHubPose3d().toPose2d();
  }

  public static double getDistanceFromRobotToHub(Pose2d robotPose) {

    if (robotPose.getTranslation().getX() > VisionConstant.MidGameMin
        && robotPose.getTranslation().getX() < VisionConstant.MidGameMax) {
      return 0;
    }
    return robotPose.getTranslation().getDistance(getTargetHubPose2d().getTranslation());
  }

  /// targetRad
  public static Rotation2d getTargetAngle(Pose2d robotPose) {
    // Vector--> robot to hub; code--> hub - robot
    // could from any degrees -> 360

    /// kinda struggle over this concept, so this link will help you
    // https://maththebeautiful.com/angle-between-points/
    // https://gamedev.stackexchange.com/questions/14602/what-are-atan-and-atan2-used-for-in-games
    // https://stackoverflow.com/questions/283406/what-is-the-difference-between-atan-and-atan2-in-c
    Translation2d diff = getTargetHubPose2d().getTranslation().minus(robotPose.getTranslation());
    Rotation2d fieldAngle = diff.getAngle().rotateBy(new Rotation2d(Math.PI));
    return fieldAngle;
  }

  //  public static double getAngleToHubRad(Pose2d robotPose) {
  //    return getAngleToHub(robotPose).getRadians();
  //  }

  /// doesn't really needed
  // error
  // https://youtu.be/55FDgyuhWTM?si=_PU5S1oCPOQ_Kzcc
  //    public static Rotation2d getTurningAngle(Pose2d robotPose) {
  //        return getAngleToHub(robotPose).minus(robotPose.getRotation());
  //    }

  //    public static double getTurningAngleRad(Pose2d robotPose) {
  //        return getTurningAngle(robotPose).getRadians();
  //    }
}
