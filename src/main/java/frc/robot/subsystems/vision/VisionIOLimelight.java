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
  private final NetworkTable LimelightFrontTable =
      NetworkTableInstance.getDefault().getTable(VisionConstant.LIMELIGHT_FRONT);
  private final NetworkTable LimelightBackTable =
      NetworkTableInstance.getDefault().getTable(VisionConstant.LIMELIGHT_BACK);
  public final Supplier<Pose2d> poseSupplier;

  // get that pose for me
  public VisionIOLimelight(Supplier<Pose2d> poseSupplier) {
    this.poseSupplier = poseSupplier;
  }

  @Override
  // getting two inputs
  public void updateInputs(VisionIOInputs frontCamInputs, VisionIOInputs backCamInputs) {
    // we use the method, give it the variable of its wanted type
    readCameraData(LimelightFrontTable, frontCamInputs, VisionConstant.LIMELIGHT_FRONT);
    readCameraData(LimelightBackTable, backCamInputs, VisionConstant.LIMELIGHT_BACK);
  }

  // LimelightHelper basically uses data from NetworkTables, and turn it into simple and easier to
  // write codes.
  // However, we do receive networktable first

  /// inputs.tx = LimelightHelpers.getTX("FrontCam");
  // equals
  /// inputs.tx = table.getEntry("tx").getDouble(0);

  // SO, you can choose to do tables, OR limelighthelper. Either way

  //            METHOD     TYPE     VARIABLES   TYPE         VARIABLES
  private void readCameraData(NetworkTable table, VisionIOInputs inputs, String cameraName) {
    // DON'T CHANGE ANY NAMING STUFF, AFTER THIS LINE OF CODE!!!!!!!!!!!!!!!!!!!!! SAYING YOU, AIDAN

    inputs.cameraConnected =
        LimelightHelpers.getLimelightNTTableEntry(cameraName, "CameraIsConnected").exists();
    inputs.tx = LimelightHelpers.getTX(cameraName);
    inputs.ty = LimelightHelpers.getTY(cameraName);
    inputs.ta = LimelightHelpers.getTA(cameraName);
    inputs.fiducialID = LimelightHelpers.getFiducialID(cameraName);
    inputs.hasTarget = LimelightHelpers.getTV(cameraName);
    inputs.estimatedRobotPose = poseSupplier.get();

    LimelightHelpers.SetRobotOrientation(
        "limelightFront", inputs.estimatedRobotPose.getRotation().getDegrees(), 0, 0, 0, 0, 0);

    if (!inputs.hasTarget) {
      inputs.hasMegaTag2 = false;
      return;
    }
    // in here, visionPose is calculated
    var megaTag2Results = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(cameraName);
    if (megaTag2Results.tagCount >= 2) {
      inputs.hasMegaTag2 = true;
      inputs.megaTagPose = megaTag2Results.pose;
      inputs.tagCount = megaTag2Results.tagCount;
      inputs.timestamp = megaTag2Results.timestampSeconds;
      inputs.latency = megaTag2Results.latency;
      return;
    }

    var megaTag1Results = LimelightHelpers.getBotPoseEstimate_wpiBlue(cameraName);
    if (megaTag1Results.tagCount >= 1) {
      inputs.hasMegaTag2 = false;
      inputs.megaTagPose = megaTag1Results.pose;
      inputs.tagCount = megaTag1Results.tagCount;
      inputs.timestamp = megaTag1Results.timestampSeconds;
      inputs.latency = megaTag1Results.latency;
      return;
    }
    inputs.hasMegaTag2 = false;
    return;
  }
}

// Start of inputs
//    @AutoLog
//    class VisionIOLimelightInputs{
//         SwerveModulePosition swerveModulePosition = new SwerveModulePosition[]{
//                 frontLeft.getPosition(),
//                 frontRight.getPosition(),
//                 backLeft.getPosition(),
//                 backRight.getPosition()}
//         double tx = LimelightHelpers.getTX(""); //offset in x direction'
//         //tx>0 right
//         //tx<0 left
//         //best: tx=0
//         double ty = LimelightHelpers.getTY(""); //offset in y direction
//         double ta = LimelightHelpers.getTA(""); //target area 0-100%
//         boolean hasTarget = LimelightHelpers.getTV("");// whether or not the camera has a
// target location
//         double aprilTagID = 0;// turn into false if we decide to use MegaTag1
//         // setting up the measured values of the camera to be set in the table//
//         LimelightHelpers.setFiducial3DOffset(//forward offset,Side offset,Height offset)
//                 LimelightHelpers .setCameraPose_RobotSpace("",//forward offset,Side
// offset,Height offset,Roll,Pitch,Yaw)
// }

// Start of outputs
// @Override
// public void periodic() {
//
//    public Pose3d estimatedRobotOrientation = LimelightHelpers.SetRobotOrientation("limelight",
// poseEstimator.getEstimatedPosition().getRotation().getDegrees(), 0, 0, 0, 0, 0);
//          // above code needs Drive System for robot orientation.
//
//     }
