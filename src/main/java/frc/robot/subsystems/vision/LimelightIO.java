package frc.robot.subsystems.vision.VisionIO.java;

public interface LimelightIO implements VisionIO{
//Start of inputs
     SwerveModulePosition swerveModulePosition = new SwerveModulePosition[]{
             frontLeft.getPosition(),
             frontRight.getPosition(),
             backLeft.getPosition(),
             backRight.getPosition()}
     double tx = LimelightHelpers.getTX(""); //offset in x direction'
     //tx>0 right
     //tx<0 left
     //best: tx=0
     double ty = LimelightHelpers.getTY(""); //offset in y direction
     double ta = LimelightHelpers.getTA(""); //target area 0-100%
     boolean hasTarget = LimelightHelpers.getTV("");// whether or not the camera has a target location
     double aprilTagID = 0;// turn into false if we decide to use MegaTag1
     // setting up the measured values of the camera to be set in the table//

//Start of outputs
@Override
public void periodic() {

          LimelightHelpers.SetRobotOrientation("limelight", m_poseEstimator.getEstimatedPosition().getRotation().getDegrees(), 0, 0, 0, 0, 0);
          // above code needs Drive System for robot orientation.
     }