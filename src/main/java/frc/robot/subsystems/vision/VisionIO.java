package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import org.littletonrobotics.junction.AutoLog;

import java.util.function.Supplier;

public interface VisionIO {
    @AutoLog
    class VisionIOInputs {
        public boolean cameraConnected;
        public boolean hasTarget;

        public double limelightHeart;

        //crosshairs
        public double tx;
        public double ty;
        public double ta;

        public boolean hasMegaTag2;

        // this is the estimated vision pose
        public Pose2d megaTagPose; // can be either megatag 1 or 2
        public double timestamp;
        public int tagCount;
        public double latency;

        public double[] rawStdDev;
        public int[] rawAprilTagID;
        public double minAmbiguity;
    }

    // name doesn't matter here
    default void setPipeline(String cameraName, int pipelineNumber) {
    }

    default void setRobotOrientation(String cameraName, double headingDegs) {
    }

    default void setIMUMode(String cameraName, int mode) {
    }

    default void setIMUAssistAlpha(String cameraName, double alpha) {
    }

    default void updateInputs(VisionIOInputs camOne, VisionIOInputs camTwo) {
    }
}
