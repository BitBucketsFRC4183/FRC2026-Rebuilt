package frc.robot.subsystems.vision;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.LimelightHelpers;

public class VisionIOSim implements VisionIO {
  public final String BitBucketsCamera = "limelight";
  public final String limelightName = "BitBucketsCamera";
  NetworkTable limelight = NetworkTableInstance.getDefault().getTable(BitBucketsCamera);

  //  public VisionIO visionIO;
  //  public VisionIOInputs visionIOInputs;

  // SwerveModulePosition swerveModulePosition = new SwerveModulePosition{
  // frontLeft.getPosition(),
  // frontRight.getPosition(),
  // backLeft.getPosition(),
  // backRight.getPosition()}

  //  @AutoLogOutput double tx = LimelightHelpers.getTX(""); // offset in x direction'
  //  // tx>0 right
  //  // tx<0 left
  //  // best: tx=0
  //  @AutoLogOutput double ty = LimelightHelpers.getTY(""); // offset in y direction
  //  @AutoLogOutput double ta = LimelightHelpers.getTA(""); // target area 0-100%
  //
  //  @AutoLogOutput
  //  boolean hasTarget = LimelightHelpers.getTV(""); // whether or not the camera has a target
  // location
  //
  //  @AutoLogOutput double aprilTagID = 0; // turn into false if we decide to use MegaTag1
  // setting up the measured values of the camera to be set in the table//
  // LimelightHelpers.setFiducial3DOffset(forward offset,Side offset,Height offset)
  // LimelightHelpers .setCameraPose_RobotSpace("",forward offset,Side offset,Height
  // offset,Roll,Pitch,Yaw)
  NetworkTableEntry txEntry = limelight.getEntry("tx");
  NetworkTableEntry tyEntry = limelight.getEntry("ty");
  NetworkTableEntry tvEntry = limelight.getEntry("tv");
  NetworkTableEntry taEntry = limelight.getEntry("ta");
  NetworkTableEntry fiducialIDEntry = limelight.getEntry("fiducialID");
  NetworkTableEntry robotPoseEntry = limelight.getEntry("Pose2d");

  public void updateInputs(VisionIOInputs inputs) {

    // Connection need rewrite, bad!
    inputs.cameraConnected =
        LimelightHelpers.getLimelightNTTableEntry(BitBucketsCamera, "CameraIsConnected").exists();

    inputs.tx = txEntry.getDouble(0.0);
    inputs.ty = tyEntry.getDouble(0.0);
    inputs.ta = taEntry.getDouble(0.0);
    inputs.tv = tvEntry.getDouble(0.0);

    inputs.fiducialID = fiducialIDEntry.getDouble(0.0);

    inputs.hasTarget = LimelightHelpers.getTV(limelightName);
    inputs.robotPose = LimelightHelpers.getBotPose2d(limelightName);
    //    visionIO.updateInputs(visionIOInputs);

    //    Logger.recordOutput("VisionSim/tx", visionIOInputs.tx);
    //    Logger.recordOutput("VisionSim/ty", visionIOInputs.ty);
    //    Logger.recordOutput("VisionSim/ta", visionIOInputs.ta);
    //    Logger.recordOutput("VisionSim/hasTarget", visionIOInputs.hasTarget);
    //    Logger.recordOutput("VisionSim/aprilTagID", visionIOInputs.aprilTagID);
    if (!inputs.hasTarget) {
      inputs.hasMegaTag2 = false;
      return;
    }
    // in here, visionPose is calculated from
    var megaTag2Results = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);
    if (megaTag2Results.tagCount >= 2) {
      inputs.hasMegaTag2 = true;
      inputs.megaTagPose = megaTag2Results.pose;
      inputs.tagCount = megaTag2Results.tagCount;
      inputs.timestamp = megaTag2Results.timestampSeconds;
      inputs.latency = megaTag2Results.latency;
      return;
    }

    var megaTag1Results = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName);
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
  // 99% sure the code above doesn't belong here. Probably need to move it somewhere else (like
  // command)
  // Pose3d estimatedRobotOrientation = LimelightHelpers.SetRobotOrientation("limelight",
  // poseEstimator.getEstimatedPosition().getRotation().getDegrees(), 0, 0, 0, 0, 0);
  // above code needs Drive System for robot orientation.
