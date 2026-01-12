package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.LimelightHelpers;
import org.littletonrobotics.junction.AutoLog;

public interface VisionIO {
    @AutoLog
    public class VisionIOInputs{
        public boolean cameraConnected = false;
        public boolean hasTarget = false;
        public double tx = 0.0;
        public double ty = 0.0;
        public double ta = 0.0;

        public double fiducialID = -1;
        public boolean hasAprilTag = false;
        public Pose2d robotPose = new Pose2d();
    }


    void updateInputs(VisionIOInputs inputs);

}
