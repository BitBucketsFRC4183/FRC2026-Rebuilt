package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.LimelightHelpers;
import frc.robot.constants.VisionConstants;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Set;

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
        LimelightHelpers.PoseEstimate megaTag2Results =
            LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(cameraName);

        if (megaTag2Results != null && inputs.tagCount >= 0) {
          inputs.hasMegaTag2 = true;
          inputs.megaTagPose = megaTag2Results.pose;
          inputs.tagCount = megaTag2Results.tagCount;
          inputs.timestamp = megaTag2Results.timestampSeconds;
          inputs.latency = megaTag2Results.latency;
          return;
        }

        inputs.tx = LimelightHelpers.getTX(cameraName);
        inputs.ty = LimelightHelpers.getTY(cameraName);
        inputs.ta = LimelightHelpers.getTA(cameraName);

        var rawFiducial = LimelightHelpers.getRawFiducials(cameraName);
        inputs.minAmbiguity = getMinAmbiguity(rawFiducial);
        //        inputs.rawAprilTagID = getAprilTagIDs(rawFiducial);

        DoubleArraySubscriber stddevs =
            table.getDoubleArrayTopic("stddevs").subscribe(new double[] {});
        inputs.rawStdDev = stddevs.get();

        DoubleArraySubscriber megatag2Subscriber =
            table.getDoubleArrayTopic("botpose_orb_wpiblue").subscribe(new double[] {});

        ///
        Set<Integer> tagIds = new HashSet<>();
        List<PoseObservation> poseObservations = new LinkedList<>();

        // if no target, continue
        for (var rawSample : megatag2Subscriber.readQueue()) {
          if (rawSample.value.length == 0) continue;

          // skip headings, every 7 elements represent a id, so then if there are more info, the
          // loop continues
          for (int i = 11; i < rawSample.value.length; i += 7) {
            tagIds.add((int) rawSample.value[i]);
          }
          poseObservations.add(
              new PoseObservation(
                  // 3D pose estimate
                  parsePose(rawSample.value),

                  // Tag count
                  (int) rawSample.value[7]

                  // Average tag distance
                  ));
        }

        // Save pose observations to inputs object
        inputs.rawPoseObservations = new PoseObservation[poseObservations.size()];
        for (int i = 0; i < poseObservations.size(); i++) {
          inputs.rawPoseObservations[i] = poseObservations.get(i);
        }

        // Save tag IDs to inputs objects
        inputs.rawTagIds = new int[tagIds.size()];
        int i = 0;
        for (int id : tagIds) {
          inputs.rawTagIds[i++] = id;
        }

        //        System.out.println(inputs.rawStdDev);
        inputs.crosshairs = table.getEntry("crosshairs").getDoubleArray(new double[4]);

      } catch (Exception e) {
        System.err.println("Error processing Limelight data: " + e.getMessage());
      }
    }
  }

  private static Pose3d parsePose(double[] rawLLArray) {
    return new Pose3d(
        rawLLArray[0],
        rawLLArray[1],
        rawLLArray[2],
        new Rotation3d(
            Units.degreesToRadians(rawLLArray[3]),
            Units.degreesToRadians(rawLLArray[4]),
            Units.degreesToRadians(rawLLArray[5])));
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
    LimelightHelpers.SetRobotOrientation_NoFlush(cameraName, headingDegs, 0, 0, 0, 0, 0);
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
