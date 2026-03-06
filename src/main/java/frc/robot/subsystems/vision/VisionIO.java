package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.constants.VisionConstant;
import org.littletonrobotics.junction.AutoLog;

public interface VisionIO {
  @AutoLog
  class VisionIOInputs {
    public boolean cameraConnected;
    public boolean hasTarget;

    public double limelightHeart;

    // crosshairs
    public double tx;
    public double ty;
    public double ta;
    public double[] crosshairs = new double[4];

    public boolean hasMegaTag2;

    // this is the estimated vision pose
    public Pose2d megaTagPose; // can be either megatag 1 or 2
    public double timestamp;
    public int tagCount;
    public double latency;

    public double[] rawStdDev = new double[12];
    public int[] rawAprilTagID = new int[VisionConstant.numAprilTagWillVisualize];
    public double minAmbiguity;
  }

  // name doesn't matter here
  default void setPipeline(String cameraName, int pipelineNumber) {}

  default void setRobotOrientation(String cameraName, double headingDegs) {}

  default void setIMUMode(String cameraName, int mode) {}

  default void setIMUAssistAlpha(String cameraName, double alpha) {}

  default void updateInputs(VisionIOInputs camOne, VisionIOInputs camTwo) {}
}
