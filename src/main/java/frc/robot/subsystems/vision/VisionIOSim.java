package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.networktables.NetworkTableEntry;
import frc.robot.LimelightHelpers;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;


public class VisionIOSim {
    public VisionIO visionIO;
    public VisionIOInputs visionIOInputs;

                //SwerveModulePosition swerveModulePosition = new SwerveModulePosition{
               // frontLeft.getPosition(),
                //frontRight.getPosition(),
                //backLeft.getPosition(),
                //backRight.getPosition()}

                @AutoLogOutput
        double tx = LimelightHelpers.getTX(""); //offset in x direction'
        //tx>0 right
        //tx<0 left
        //best: tx=0
        @AutoLogOutput
        double ty = LimelightHelpers.getTY(""); //offset in y direction
    @AutoLogOutput
        double ta = LimelightHelpers.getTA(""); //target area 0-100%
    @AutoLogOutput
        boolean hasTarget = LimelightHelpers.getTV("");// whether or not the camera has a target location
    @AutoLogOutput
        double aprilTagID = 0;// turn into false if we decide to use MegaTag1
        // setting up the measured values of the camera to be set in the table//
     //LimelightHelpers.setFiducial3DOffset(forward offset,Side offset,Height offset)
            // LimelightHelpers .setCameraPose_RobotSpace("",forward offset,Side offset,Height offset,Roll,Pitch,Yaw)

//    public void updateInputs(VisionIOInputs inputs) {
//        visionIO.updateInputs(visionIOInputs);
//
//
//        Logger.recordOutput("VisionSim/tx", visionIOInputs.tx);
//        Logger.recordOutput("VisionSim/ty", visionIOInputs.ty);
//        Logger.recordOutput("VisionSim/ta", visionIOInputs.ta);
//        Logger.recordOutput("VisionSim/hasTarget", visionIOInputs.hasTarget);
//        Logger.recordOutput("VisionSim/aprilTagID", visionIOInputs.aprilTagID);;;
//    }
        //99% sure the code above doesn't belong here. Probably need to move it somewhere else (like command)
        //Pose3d estimatedRobotOrientation = LimelightHelpers.SetRobotOrientation("limelight", poseEstimator.getEstimatedPosition().getRotation().getDegrees(), 0, 0, 0, 0, 0);
        // above code needs Drive System for robot orientation.

}