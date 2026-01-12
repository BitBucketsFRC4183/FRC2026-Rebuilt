package frc.robot.subsystems.vision;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.vision.VisionIO;
import org.littletonrobotics.junction.AutoLog;

public interface VisionIOLimelight extends VisionIO {

    public final String BitBucketsCamera  = "limelight";



    //Start of inputs
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
//         boolean hasTarget = LimelightHelpers.getTV("");// whether or not the camera has a target location
//         double aprilTagID = 0;// turn into false if we decide to use MegaTag1
//         // setting up the measured values of the camera to be set in the table//
//         LimelightHelpers.setFiducial3DOffset(//forward offset,Side offset,Height offset)
//                 LimelightHelpers .setCameraPose_RobotSpace("",//forward offset,Side offset,Height offset,Roll,Pitch,Yaw)
//}

//Start of outputs
//@Override
//public void periodic() {
//
//    public Pose3d estimatedRobotOrientation = LimelightHelpers.SetRobotOrientation("limelight", poseEstimator.getEstimatedPosition().getRotation().getDegrees(), 0, 0, 0, 0, 0);
//          // above code needs Drive System for robot orientation.
//
//     }
    @Override
    public default void updateInputs(VisionIOInputs inputs){
        inputs.cameraConnected = LimelightHelpers.getLimelightNTTableEntry("BitBucketsCamera", "CameraIsConnected").exists();
        inputs.tx = LimelightHelpers.getTX("BitBucketsCamera");
        inputs.ty = LimelightHelpers.getTY("BitBucketsCamera");
        inputs.ta = LimelightHelpers.getTA("BitBucketsCamera");

        inputs.fiducialID = LimelightHelpers.getFiducialID("BitBucketsCamera");
        inputs.hasAprilTag = inputs.fiducialID != -1;

        inputs.hasTarget = LimelightHelpers.getTV("BitBucketsCamera");
        inputs.robotPose = LimelightHelpers.getBotPose2d("BitBucketsCamera");

    }
}