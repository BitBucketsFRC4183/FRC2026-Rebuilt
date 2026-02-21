package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Rotation2d;
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

  private Supplier<Rotation2d> headingSupplier;

  // get that pose for me
  public VisionIOLimelight(Supplier<Rotation2d> headingSupplier) {
    this.headingSupplier = headingSupplier;
  }

  @Override
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

  private void readCameraData(String cameraName, VisionIOInputsAutoLogged inputs) {
    /// basics
    // drive pose

    LimelightHelpers.SetRobotOrientation(
        cameraName, headingSupplier.get().getDegrees(), 0, 0, 0, 0, 0);

    inputs.cameraConnected = LimelightHelpers.getLimelightNTTable(cameraName) != null;
    // fiducialid is double
    inputs.aprilTagIDNumber = (int) LimelightHelpers.getFiducialID(cameraName);

    inputs.hasTarget = LimelightHelpers.getTV(cameraName);

    inputs.tx = LimelightHelpers.getTX(cameraName);
    inputs.ty = LimelightHelpers.getTY(cameraName);
    inputs.ta = LimelightHelpers.getTA(cameraName);

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
