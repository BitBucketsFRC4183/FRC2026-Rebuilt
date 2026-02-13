package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.constants.AprilTagLabel;

public class HopperTracker {
    /// fancy notation
    /// function that give you hub pose based on alliance color
    public static Pose3d getTargetHubPose3d() {
        var alliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue);
        return (alliance == DriverStation.Alliance.Red)
                ? AprilTagLabel.RedHubPose3d
                : AprilTagLabel.BlueHubPose3d;
    }

    public static double getDistanceFromRobotToHub(Pose2d robotPose) {
        Pose2d hubPose2d = getTargetHubPose3d().toPose2d();
return robotPose.getTranslation()
        .getDistance(hubPose2d.getTranslation());
    }
}
