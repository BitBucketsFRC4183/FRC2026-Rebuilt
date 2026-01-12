package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import org.littletonrobotics.junction.AutoLog;

public interface VisionIO {
    @AutoLog
    public class VisionIOInputs{
        public boolean cameraConnected = false;
        boolean hasTarget = false;
        public double tx = 0.0;
        public double ty = 0.0;
        public double ta = 0.0;

        boolean hasAprilTag = false;
        boolean robotPose = new pose2D();
    }


    void updateInputs(VisionIOInputs inputs);

}
