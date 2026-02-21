package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.LimelightHelpers;
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

  private final Supplier<Rotation2d> headingSupplier;
  private double lastGyro_degs;
  private double lastGyro_timestamp;
  private final String cameraName;

  public VisionIOLimelight(String cameraName, Supplier<Rotation2d> headingSupplier) {
    this.headingSupplier = headingSupplier;
    this.cameraName = cameraName;
    this.lastGyro_degs = headingSupplier.get().getDegrees();
    this.lastGyro_timestamp = Timer.getFPGATimestamp();
  }

  // LimelightHelper basically uses data from NetworkTables, and turn it into simple and easier to
  // write codes.
  // However, we do receive networktable first

  /// inputs.tx = LimelightHelpers.getTX("FrontCam");
  // equals
  /// inputs.tx = table.getEntry("tx").getDouble(0);

  // SO, you can choose to do tables, OR limelighthelper. Either way
  @Override
  public void updateInputs(VisionIOInputsAutoLogged inputs) {
    if (DriverStation.isDisabled()) {
      LimelightHelpers.SetIMUMode(cameraName, 1);
      LimelightHelpers.SetRobotOrientation(
          cameraName, headingSupplier.get().getDegrees(), 0, 0, 0, 0, 0);
    } else {
      LimelightHelpers.SetIMUMode(cameraName, 4);
      // Set the complementary filter alpha (optional, default is 0.001, higher ext_gyro confidence
      // 0.01)
      LimelightHelpers.SetIMUAssistAlpha(cameraName, 0.001);
    }

    inputs.cameraConnected = LimelightHelpers.getLimelightNTTable(cameraName) != null;
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
    // we are gonna validate the pose here
    var megaTag2Results = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(cameraName);

    double dt = Timer.getFPGATimestamp() - lastGyro_timestamp;
    double dd = headingSupplier.get().getDegrees() - lastGyro_degs;
    double rate = Math.abs(dd / dt);

    if (megaTag2Results != null && inputs.tagCount >= 0 && rate < 360) {
      inputs.hasMegaTag2 = true;
      inputs.megaTagPose = megaTag2Results.pose;
      inputs.estimatedRobotPose = megaTag2Results.pose;
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
