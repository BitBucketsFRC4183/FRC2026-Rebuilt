package frc.robot.subsystems.vision;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.LimelightHelpers;
import frc.robot.constants.VisionConstants;

public class VisionIOLimelight implements VisionIO {

  private double lasthb = -1;
  // LIMELIGHT is constant
  // "limelight" is Networktables path

  // getTable(""), inside the "", is webUI/table name
  // name doesn't matter here as well
  private final NetworkTable limelightOneTable =
      NetworkTableInstance.getDefault().getTable(VisionConstants.LIMELIGHT_A);

  private final NetworkTable limelightTwoTable =
      NetworkTableInstance.getDefault().getTable(VisionConstants.LIMELIGHT_B);

  @Override
  public void updateInputs(VisionIOInputs camOneData, VisionIOInputs camTwoData) {
    // we use the method, give it the variable of its wanted type
    readCameraData(limelightOneTable, camOneData, VisionConstants.LIMELIGHT_A);
    readCameraData(limelightTwoTable, camTwoData, VisionConstants.LIMELIGHT_B);
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
    inputs.cameraConnected = LimelightHelpers.getHeartbeat(cameraName) != lasthb;
    lasthb = LimelightHelpers.getHeartbeat(cameraName);
    inputs.limelightHeart = LimelightHelpers.getHeartbeat(cameraName);

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
        inputs.rawAprilTagID = getAprilTagIDs(rawFiducial);

        inputs.tx = LimelightHelpers.getTX(cameraName);
        inputs.ty = LimelightHelpers.getTY(cameraName);
        inputs.ta = LimelightHelpers.getTA(cameraName);
        inputs.rawStdDev = table.getEntry("stddevs").getDoubleArray(new double[12]);
        //        System.out.println(inputs.rawStdDev);
        inputs.crosshairs = table.getEntry("crosshairs").getDoubleArray(new double[4]);

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

  private static int[] getAprilTagIDs(LimelightHelpers.RawFiducial[] unreadFiducial) {
    if (unreadFiducial == null || unreadFiducial.length == 0) {
      return new int[0];
    } else {
      int[] ids = new int[unreadFiducial.length];
      for (int i = 0; i < unreadFiducial.length; i++) {
        ids[i] = unreadFiducial[i].id;
      }
      return ids;
    }
  }

  private static double getMinAmbiguity(LimelightHelpers.RawFiducial[] unreadFiducial) {
    /// ambiguity, new!

    double minAmbiguity = 999;
    for (var readFludicial : unreadFiducial) {
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
