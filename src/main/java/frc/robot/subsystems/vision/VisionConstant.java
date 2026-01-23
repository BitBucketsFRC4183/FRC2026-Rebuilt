package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;

import java.io.IOException;

public final class VisionConstant {
  // They are all capitalized in case you didn't realize it is a constant

  // We need to change that in the settings
  public static final String LIMELIGHT_FRONT = "limelightFront";
  public static final String LIMELIGHT_BACK = "limelightBack";
  public static AprilTagFieldLayout aprilTagFieldLayout;
    static {
        try {
            aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource("2026-rebuilt-welded.json");
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
    }

    private VisionConstant() {}
}
