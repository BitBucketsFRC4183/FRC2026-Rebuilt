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

  //just for sim
  public static Transform3d robotToFrontCam = new Transform3d(0, 0, 0, new Rotation3d());
  public static Transform3d robotToBackCam = new Transform3d(0, 0, 0, new Rotation3d());

  public static final int kExpectedStdDevArrayLength = 12;
  public static final double kLargeVariance = 1e6;

  //TODO need tune
  //max readable tag
  public static final double maxDistanceFromRobotToApril = 17.0;
  public static final double kTagMinAreaForSingleTagMegatag = 1.0;
  public static final double kDefaultNormThreshold = 1.0;
  public static final double kMinAmbiguityToFlip = 666;

  //TODO need measure & test
  //-0.677 = from center of hub minus the center robot, it is about this much allowance
  // this is a valid estimation lol
  public static final double MidGameAllowance = -0.677;

  /// TODO need Distance offset from shooter
  public static final double baseX = 0.1;
  public static final double baseY = 0.1;
  public static final double baseTheta = 5.0;




  private VisionConstant() {}
}
