package frc.robot.constants;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import java.io.IOException;

public final class VisionConstant {
  // They are all capitalized in case you didn't realize it is a constant

  // We need to change that in the settings
  // not pipeline name!!!!!
  public static final String LIMELIGHT_FRONT = "limelight-side";
  public static final String LIMELIGHT_FRONT_SHOOTER = "limelight-frontShooter";

  public static AprilTagFieldLayout aprilTagFieldLayout;

  static {
    try {
      aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource("2026-rebuilt-welded.json");
    } catch (IOException e) {
      throw new RuntimeException(e);
    }
  }

  // just for sim
  public static Transform3d robotToFrontCam = new Transform3d(0, 0, 0, new Rotation3d());
  public static Transform3d robotToBackCam = new Transform3d(0, 0, 0, new Rotation3d());

  // TODO tune them
  // start high values, then slowly lower them
  public static final Matrix<N3, N1> GlobalVisionMeasurementStdDevs =
      VecBuilder.fill(0.7, 0.7, Units.degreesToRadians(180));

  public static final Matrix<N3, N1> VeryGood_VisionMeasurementStdDevs =
      VecBuilder.fill(0.4, 0.4, Units.degreesToRadians(90));
  public static final Matrix<N3, N1> Ish_VisionMeasurementStdDevs =
      VecBuilder.fill(0.3, 0.3, Units.degreesToRadians(90));

  private VisionConstant() {}
}
