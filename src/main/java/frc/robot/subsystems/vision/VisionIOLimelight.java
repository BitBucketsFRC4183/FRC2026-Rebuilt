package frc.robot.subsystems.vision;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.LimelightHelpers;
import frc.robot.constants.VisionConstant;

public class VisionIOLimelight implements VisionIO {

  // LIMELIGHT is constant
  // "limelight" is Networktables path

  // getTable(""), inside the "", is webUI/table name
  // name doesn't matter here as well
  private final NetworkTable limelightOneTable =
      NetworkTableInstance.getDefault().getTable(VisionConstant.LIMELIGHT_A);

  private final NetworkTable limelightTwoTable =
      NetworkTableInstance.getDefault().getTable(VisionConstant.LIMELIGHT_B);

  // define, create a 0.0 double array
  private static final double[] defaultStdDev =
      new double[VisionConstant.kExpectedStdDevArrayLength];

  @Override
  public void updateInputs(VisionIOInputs camOneData, VisionIOInputs camTwoData) {
    // we use the method, give it the variable of its wanted type
    readCameraData(limelightOneTable, camOneData, VisionConstant.LIMELIGHT_A);
    readCameraData(limelightTwoTable, camTwoData, VisionConstant.LIMELIGHT_B);
  }

  // LimelightHelper basically uses data from NetworkTables, and turn it into simple and easier to
  // write codes.
  // However, we do receive networktable first

  /// inputs.tx = LimelightHelpers.getTX("FrontCam"); equals inputs.tx =
  // table.getEntry("tx").getDouble(0);
  // SO, you can choose to do tables, OR limelighthelper. Either way

  //            METHOD     TYPE     VARIABLES   TYPE         VARIABLES
  private void readCameraData(NetworkTable table, VisionIOInputs inputs, String cameraName) {
    /// basics
    // drive pose
    inputs.hasTarget = LimelightHelpers.getTV(cameraName);

    if (inputs.hasTarget) {
      try {
        /** MEGA TAG 2 * */
        var megaTag2Results = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(cameraName);

        if (megaTag2Results != null && inputs.tagCount >= 0) {
          inputs.hasMegaTag2 = true;
          inputs.megaTagPose = megaTag2Results.pose;
          inputs.tagCount = megaTag2Results.tagCount;
          inputs.timestamp = megaTag2Results.timestampSeconds;
          inputs.latency = megaTag2Results.latency;
          return;
        }

        var rawFiducial = LimelightHelpers.getRawFiducials(cameraName);
        inputs.minAmbiguity = getMinAmbiguity(rawFiducial);

        inputs.cameraConnected = LimelightHelpers.getLimelightNTTable(cameraName) != null;
        inputs.tx = LimelightHelpers.getTX(cameraName);
        inputs.ty = LimelightHelpers.getTY(cameraName);
        inputs.ta = LimelightHelpers.getTA(cameraName);
        inputs.rawStdDev = table.getEntry("stddevs").getDoubleArray(defaultStdDev);

      } catch (Exception e) {
        System.err.println("Error processing Limelight data: " + e.getMessage());
      }
    }
  }

  @Override
  public void setPipeline(String cameraName, int pipelineNumber) {
    NetworkTableInstance.getDefault()
        .getTable(cameraName)
        .getEntry("pipeline")
        .setNumber(pipelineNumber);
  }

  @Override
  public void setRobotOrientation(String cameraName, double headingDegs) {
    LimelightHelpers.SetRobotOrientation(cameraName, headingDegs, 0, 0, 0, 0, 0);
  }

  @Override
  public void setIMUMode(String cameraName, int mode) {
    LimelightHelpers.SetIMUMode(cameraName, mode);
  }

  @Override
  public void setIMUAssistAlpha(String cameraName, double alpha) {
    LimelightHelpers.SetIMUAssistAlpha(cameraName, alpha);
  }

  private static double getMinAmbiguity(LimelightHelpers.RawFiducial[] UnreadReadFiducial) {
    /// ambiguity, new!

    double minAmbiguity = 999;
    for (var readFludicial : UnreadReadFiducial) {
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
