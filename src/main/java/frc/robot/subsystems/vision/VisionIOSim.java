package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.networktables.NetworkTableEntry;
import frc.robot.LimelightHelpers;
import org.littletonrobotics.junction.AutoLog;


public class VisionIOSim {
    @AutoLog
    class VisionIOSim{
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
     LimelightHelpers.setFiducial3DOffset(//forward offset,Side offset,Height offset)
             LimelightHelpers .setCameraPose_RobotSpace("",//forward offset,Side offset,Height offset,Roll,Pitch,Yaw)

        private NetworkTableEntry hasTarget;
        private NetworkTableEntry tx;
        private NetworkTableEntry ty;
        private NetworkTableEntry ta;
        private NetworkTableEntry aprilTagID;
        private NetworkTableEntry swerveModulePosition;
    }

    @Override
    public void periodic() {

        public Pose3d estimatedRobotOrientation = LimelightHelpers.SetRobotOrientation("limelight", poseEstimator.getEstimatedPosition().getRotation().getDegrees(), 0, 0, 0, 0, 0);
        // above code needs Drive System for robot orientation.
        private NetworkTableEntry estimatedRobotOrientation;
    }
    public default void updateInputs(LimelightIOInputs inputs){}
}