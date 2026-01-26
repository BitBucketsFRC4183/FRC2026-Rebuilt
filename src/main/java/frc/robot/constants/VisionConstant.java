package frc.robot.constants;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import java.io.IOException;

public final class VisionConstant {
  // They are all capitalized in case you didn't realize it is a constant

  // We need to change that in the settings
  // not pipeline name!!!!!
  public static final String LIMELIGHT_FRONT = "limelight-front";
  public static final String LIMELIGHT_BACK = "limelight-back";

  public static AprilTagFieldLayout aprilTagFieldLayout;

  static {
    try {
      aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource("2026-rebuilt-welded.json");
    } catch (IOException e) {
      throw new RuntimeException(e);
    }
  }

  public static Transform3d robotToFrontCam = new Transform3d(0, 0, 0, new Rotation3d());
  public static Transform3d robotToBackCam = new Transform3d(0, 0, 0, new Rotation3d());

  private VisionConstant() {}
}
