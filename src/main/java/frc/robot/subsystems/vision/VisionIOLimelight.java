package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.LimelightHelpers;
import frc.robot.constants.VisionConstant;

import java.util.function.Supplier;

public class VisionIOLimelight implements VisionIO {

  // LIMELIGHT is constant
  // "limelight" is Networktables path

  // getTable(""), inside the "", is webUI/table name
  // name doesn't matter here as well
  private final NetworkTable limelightOneTable = NetworkTableInstance.getDefault().getTable(VisionConstant.LIMELIGHT_FRONT);

  private final NetworkTable limelightTwoTable = NetworkTableInstance.getDefault().getTable(VisionConstant.LIMELIGHT_FRONT_SHOOTER);

  private final Supplier<Pose2d> poseSupplier;
  private VisionPoseFusion visionPoseFusion;
  //define, create a 0.0 double array
  private static final double[] defaultStdDev =
          new double[VisionConstant.kExpectedStdDevArrayLength];


  // get that pose for me
  public VisionIOLimelight(Supplier<Pose2d> poseSupplier) {

    this.poseSupplier = poseSupplier;
  }

  @Override
  public void updateInputs(VisionIOInputs camOneData, VisionIOInputs camTwoData) {
    // we use the method, give it the variable of its wanted type
    readCameraData(limelightOneTable, camOneData, VisionConstant.LIMELIGHT_FRONT);
    readCameraData(limelightTwoTable, camTwoData, VisionConstant.LIMELIGHT_FRONT_SHOOTER);
  }

  // LimelightHelper basically uses data from NetworkTables, and turn it into simple and easier to write codes.
  // However, we do receive networktable first

  /// inputs.tx = LimelightHelpers.getTX("FrontCam"); equals inputs.tx = table.getEntry("tx").getDouble(0);
  // SO, you can choose to do tables, OR limelighthelper. Either way

  //            METHOD     TYPE     VARIABLES   TYPE         VARIABLES
  private void readCameraData(NetworkTable table, VisionIOInputs inputs, String cameraName) {
    /// basics
    // drive pose
    inputs.hasTarget = LimelightHelpers.getTV(cameraName);

    if (inputs.hasTarget) {
      try {
        inputs.estimatedRobotPose = poseSupplier.get();
        LimelightHelpers.SetRobotOrientation(cameraName, inputs.estimatedRobotPose.getRotation().getDegrees(), 0, 0, 0, 0, 0);

        /** MEGA TAG 2 **/
        var megaTag2Results = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(cameraName);

        if (megaTag2Results != null && inputs.tagCount >= 0) {
          inputs.hasMegaTag2 = true;
          inputs.megaTagPose = megaTag2Results.pose;
          inputs.tagCount = megaTag2Results.tagCount;
          inputs.timestamp = megaTag2Results.timestampSeconds;
          inputs.latency = megaTag2Results.latency;
          //package them
//          visionPoseFusion = new VisionPoseFusion(inputs.megaTagPose, inputs.timestamp, inputs.visionStdDev, inputs.tagCount);
          return;
        }

        var rawFiducial = LimelightHelpers.getRawFiducials(cameraName);
        inputs.minAmbiguity = getMinAmbiguity(rawFiducial);


        inputs.cameraConnected = LimelightHelpers.getLimelightNTTable(cameraName) != null;
        inputs.tx = LimelightHelpers.getTX(cameraName);
        inputs.ty = LimelightHelpers.getTY(cameraName);
        inputs.ta = LimelightHelpers.getTA(cameraName);
        inputs.rawStdDev = table.getEntry("stddevs").getDoubleArray(defaultStdDev);

        /// log details for hopperTracker, also for testing
//        inputs.TargetHubPose2d = HopperTracker.getTargetHubPose2d();
//        inputs.DistanceFromRobotToHub = HopperTracker.getDistanceFromRobotToHub(inputs.estimatedRobotPose);
//        inputs.FieldAngleFromHubToRobot = HopperTracker.getAngleToHub(inputs.estimatedRobotPose);
        //    inputs.TurningAngle = HopperTracker.getTurningAngle(inputs.estimatedRobotPose);

      } catch (Exception e) {
        System.err.println("Error processing Limelight data: " + e.getMessage());
      }

    }

  }
  private static double getMinAmbiguity(LimelightHelpers.RawFiducial[] UnreadReadFiducial){
    /// ambiguity, new!

    double minAmbiguity = 999;
    for (var readFludicial : UnreadReadFiducial){
      minAmbiguity = Math.min(minAmbiguity, readFludicial.ambiguity);
    }
    return minAmbiguity;
  }

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
