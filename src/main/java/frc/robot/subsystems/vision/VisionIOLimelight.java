package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.LimelightHelpers;
import java.util.Map;
import java.util.function.Supplier;

public class VisionIOLimelight implements VisionIO {

  // LIMELIGHT is constant
  // "limelight" is Networktables path

  // getTable(""), inside the "", is webUI/table name
  //  private final NetworkTable limelight_FrontCam =
  // NetworkTableInstance.getDefault().getTable(VisionConstant.LIMELIGHT_FRONT);
  //
  //  private final NetworkTable limelight_FrontShooterCam =
  // NetworkTableInstance.getDefault().getTable(VisionConstant.LIMELIGHT_FRONT_SHOOTER);

  private final Supplier<Pose2d> poseSupplier;

  // get that pose for me
  public VisionIOLimelight(Supplier<Pose2d> poseSupplier) {

    this.poseSupplier = poseSupplier;
  }

  @Override
  //
  public void updateInputs(Map<String, VisionIOInputsAutoLogged> cameraInputsAll) {
    // we use the method, give it the variable of its wanted type
    for (Map.Entry<String, VisionIOInputsAutoLogged> entry : cameraInputsAll.entrySet()) {
      String cameraName = entry.getKey();
      VisionIOInputsAutoLogged inputs = entry.getValue();
      readCameraData(cameraName, inputs);
    }
  }

  // LimelightHelper basically uses data from NetworkTables, and turn it into simple and easier to
  // write codes.
  // However, we do receive networktable first

  /// inputs.tx = LimelightHelpers.getTX("FrontCam");
  // equals

  /// inputs.tx = table.getEntry("tx").getDouble(0);

  // SO, you can choose to do tables, OR limelighthelper. Either way

  //            METHOD     TYPE     VARIABLES   TYPE         VARIABLES
  private void readCameraData(String cameraName, VisionIOInputsAutoLogged inputs) {
    /// basics
    // drive pose
    inputs.estimatedRobotPose = poseSupplier.get();

    LimelightHelpers.SetRobotOrientation(cameraName, inputs.estimatedRobotPose.getRotation().getDegrees(), 0, 0, 0, 0, 0);

    inputs.cameraConnected = LimelightHelpers.getLimelightNTTable(cameraName)!=null;
    // fiducialid is double
    inputs.aprilTagIDNumber = (int) LimelightHelpers.getFiducialID(cameraName);

    inputs.hasTarget = LimelightHelpers.getTV(cameraName);

    inputs.tx = LimelightHelpers.getTX(cameraName);
    inputs.ty = LimelightHelpers.getTY(cameraName);
    inputs.ta = LimelightHelpers.getTA(cameraName);

    /// log details for hopperTracker, also for testing
    inputs.TargetHubPose2d = HopperTracker.getTargetHubPose2d();
    inputs.DistanceFromRobotToHub =
        HopperTracker.getDistanceFromRobotToHub(inputs.estimatedRobotPose);
    inputs.FieldAngleFromHubToRobot = HopperTracker.getAngleToHub(inputs.estimatedRobotPose);
    //    inputs.TurningAngle = HopperTracker.getTurningAngle(inputs.estimatedRobotPose);


    //    var ApriltagResults = LimelightHelpers.getRawFiducials(cameraName);

    //    if (!inputs.hasTarget) {
    //      inputs.hasMegaTag2 = false;
    //      return;
    //    }

    /** MEGA TAG 2 */
    var megaTag2Results = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(cameraName);

    if (megaTag2Results != null && inputs.tagCount >= 0) {
      inputs.hasMegaTag2 = true;
      inputs.megaTagPose = megaTag2Results.pose;
      inputs.tagCount = megaTag2Results.tagCount;
      inputs.timestamp = megaTag2Results.timestampSeconds;
      inputs.latency = megaTag2Results.latency;
      return;
    }

    /** MEGA TAG 1 */
    //    var megaTag1Results = LimelightHelpers.getBotPoseEstimate_wpiBlue(cameraName);
    //
    //    if (megaTag1Results != null && megaTag1Results.tagCount >= 1) {
    //      inputs.hasMegaTag2 = false;
    //      inputs.megaTagPose = megaTag1Results.pose;
    //      inputs.tagCount = megaTag1Results.tagCount;
    //      inputs.timestamp = megaTag1Results.timestampSeconds;
    //      inputs.poseAmbiguity = getMinAmbiguity(megaTag1Results);
    //      return;
    //    }
    //
    //    inputs.hasMegaTag2 = false;
    //    return;

  }
  //  private static double getMinAmbiguity(LimelightHelpers.PoseEstimate fludicalResults){
  //    /// ambiguity, new!
  //
  //    double minAmbiguity = 999;
  //    for (var UnreadRadFludicial : fludicalResults.rawFiducials){
  //      minAmbiguity = Math.min(minAmbiguity, UnreadRadFludicial.ambiguity);
  //    }
  //    return minAmbiguity;
  //  }

}

/// @@@@@@@@@@@@@@@@@@@@@@@@@@@@@++*@@@#+++++++#@@@@@@@@@@@@@@@@@@@@@@@@
/// @@@@@@@@@@@@@@@@@@@@@@@@@@@*+*++#*+++++++*@@@%%@@@@@@@@@@@@@@@@@@@@@
/// @@@@@@@@@@@@@@@@@@@@@@@@@@@@@#+++++++++++#@@@@@@@@@@@@@@@@@@@@@@@@@@
/// @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@*+++++++++++*#@@@@@@@@@@@@@@@@@@@@@@
/// @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@++++++++++++++**@@@@@@@@@@@@@@@@@@
/// @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@*+#@@#++++++++++++++*@@@@@@@@@@@@@@@@
/// @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@*+*@@%+++++++++++%@@@@@@@@@@@@@@@@@@@
/// @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@**++++++*@@@@@@@@@@@@@@@@@@@@@@@@@
/// @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@*++++++++%@@%+++@@@@@@@@@@@@@@@@@@
/// @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@++#++++++**++++@@@@@@@@@@@@@@@@@@
/// @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@*#@@*++++++++++#@@@@@@@@@@@@@@@@@
/// @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@+%@@@*+++++++++#@@@@%**%@@@@@@@@@@
/// @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@*@@@@++++++++++++++++++###**+++++++
/// @@@@@*+++++++++++*****###%@@@@@*+%@@*+++++++++++++++++++++++++++++++
/// @@@@@@@@@@*+*#*++++++***++++++++++++++++++++++++++++++++++++++++++++
/// @@@@@@@@@@@@@@@@@@@@%*++@@@@@@@@@@+++++++++++++++**+++++++++++++++++
/// @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@%#%%@@@@@#++++@@@@@@#*++++++++++++
/// @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@+#*++@@@@@@@@@@@@@@%#**+
/// @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@%@@##@@@@@@@@@@@@@@@@@@@
/// @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
