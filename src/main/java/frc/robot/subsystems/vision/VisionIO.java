package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.LimelightHelpers;
import org.littletonrobotics.junction.AutoLog;

public interface VisionIO {
    void updateInputs(VisionIOInputs inputs);
}
//@AutoLog
//public class VisionIOInputs{
//    public boolean cameraConnected;
//    public boolean hasTarget;
//    public double tx;
//    public double ty;
//    public double ta;
//
//    public double fiducialID;
//    public boolean hasAprilTag;
//    public Pose2d robotPose;
//
//    public double latency;
//}