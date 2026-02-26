package frc.robot.constants;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;

import java.io.IOException;

public final class VisionConstant {
    // They are all capitalized in case you didn't realize it is a constant
    // We need to change that in the settings

/// **********
/// **********
/*
  FieldDimensionsX LONG = 651.2in/16.54m;
  FieldDimensionsY SHORT = 317.7in/8.07m;

  AllianceZoneY = 317.7in/8.07m;
  AllianceZoneX/~HubToDriverStation = 158.6in/4.03m;
---> ExcludeClimb = 2.779m;

  ROBOT
  +x -> intake
  -x -> shooter

  AlongX / intakeToShooter = 26.5in/0.6731m;
  Y = 28in/0.7112m;

  CAMERA A


  PIPELINE RESULTS/APPROXIMATED STABLE APRILTAG DETECTION DISTANCE/CPU USAGE

  DEFAULT_AprilTagTuning = 2.7432m; SHOULD COVER WHOLE ALLIANCE ZONE; 60.2%ish
  DEFAULT_DriverCamera; 22.7% - 25%
  DEFAULT_OFF; technically not off, but close to off; lower frame rate; 6%

 */
    /// **********
    /// **********
    public static final String LIMELIGHT_FRONT = "limelight-side";
    public static final String LIMELIGHT_FRONT_SHOOTER = "limelight-frontShooter";

    public static final int PIPELINE_DEFAULT_AprilTagTuning = 0;
    public static final int PIPELINE_DEFAULT_DriverCamera = 1;
    public static final int PIPELINE_DEFAULT_OFF = 2;

    public static final int PIPELINE_Autonomous = 3;
    public static final int PIPELINE_Teleop = 4;


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

    public static final int kExpectedStdDevArrayLength = 12;
    public static final double kLargeVariance = 1e6;

    // TODO need tune: max readable tag
    public static final double complementaryFilterAlphaIMU = 0.001;
    public static final double maxDistanceFromRobotToApril = 17.0;
    public static final double kTagMinAreaForSingleTagMegatag = 3.0;
    public static final double kDefaultNormThreshold = 1.0;
    public static final double kMinAmbiguityToFlip = 666;

    // TODO need measure & test
    /*
     -0.677 = from center of hub minus the center robot, it is about this much allowance.
     this is a valid estimation lol
     */
    public static final double MidGameAllowance = -0.677;

    private VisionConstant() {
    }
}
