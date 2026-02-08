package frc.robot.constants;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.DriverStation;

import static frc.robot.constants.VisionConstant.aprilTagFieldLayout;

public class AprilTagLabel {
    private AprilTagLabel() {}
    public static final int RED_RIGHT_BELOW_HUB = 10;
    public static final int BLUE_RIGHT_BELOW_HUB = 26;
    public static final int RED_LITTLE_SIDE_BELOW_HUB =9;
    public static final int BLUE_LITTLE_SIDE_BELOW_HUB =25;
    //offset, it is the center of hopper
    public static final Transform3d tagToHub3d = new Transform3d(
            new Translation3d(0.2,0, 0.6096),
            new Rotation3d()
    );
    public static final Pose3d RedHubPose3d = aprilTagFieldLayout.getTagPose(RED_RIGHT_BELOW_HUB).plus(tagToHub3d);
    public static final Pose3d BlueHubPose3d = aprilTagFieldLayout.getTagPose(BLUE_RIGHT_BELOW_HUB).plus(tagToHub3d) ;
    Pose3d targetHubPose = (DriverStation.Alliance.Red) ? RedHubPose3d : BlueHubPose3d;
}
