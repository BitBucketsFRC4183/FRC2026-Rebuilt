package frc.robot.subsystems.vision;

import frc.robot.LimelightHelpers;

public class VisionIOLimelight implements VisionIO {

  public final String BitBucketsCamera = "limelight";

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
  @Override
  public void updateInputs(VisionIOInputs inputs) {
    inputs.cameraConnected =
        LimelightHelpers.getLimelightNTTableEntry("BitBucketsCamera", "CameraIsConnected").exists();
    inputs.tx = LimelightHelpers.getTX("BitBucketsCamera");
    inputs.ty = LimelightHelpers.getTY("BitBucketsCamera");
    inputs.ta = LimelightHelpers.getTA("BitBucketsCamera");

    inputs.fiducialID = LimelightHelpers.getFiducialID("BitBucketsCamera");

    inputs.hasTarget = LimelightHelpers.getTV("BitBucketsCamera");
    inputs.robotPose = LimelightHelpers.getBotPose2d("BitBucketsCamera");

    // if there is no target, then don't continue other inputs
    if (!inputs.hasTarget) {
      inputs.hasMegaTag2 = false;
      return;
    }
    // in here, visionPose is calculated from
    var megaTag2Results = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("BitBucketsCamera");
    if (megaTag2Results.tagCount >= 2) {
      inputs.hasMegaTag2 = true;
      inputs.megaTagPose = megaTag2Results.pose;
      inputs.tagCount = megaTag2Results.tagCount;
      inputs.timestamp = megaTag2Results.timestampSeconds;
      inputs.latency = megaTag2Results.latency;
      return;
    }

    var megaTag1Results = LimelightHelpers.getBotPoseEstimate_wpiBlue("BitBucketsCamera");
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
