package frc.robot.constants;

import static frc.robot.constants.VisionConstant.aprilTagFieldLayout;

import edu.wpi.first.math.geometry.*;

public class AprilTagLabel {
  private AprilTagLabel() {}

  public static final int RED_RIGHT_BELOW_HUB = 10;
  public static final int BLUE_RIGHT_BELOW_HUB = 26;
  public static final int RED_LITTLE_SIDE_BELOW_HUB = 9;
  public static final int BLUE_LITTLE_SIDE_BELOW_HUB = 25;

  // offset, it is the center of hopper
  // 2foot height from apriltag to hopper
  // 23.5inch from center of hopper

  // TODO need to measure

  public static final Pose3d RedHubPose3d =
      aprilTagFieldLayout
          .getTagPose(RED_RIGHT_BELOW_HUB)
          .get()
          .plus(VisionConstant.tagToHub3d.inverse());

  public static final Pose3d BlueHubPose3d =
      aprilTagFieldLayout
          .getTagPose(BLUE_RIGHT_BELOW_HUB)
          .get()
          .plus(VisionConstant.tagToHub3d.inverse());
}
