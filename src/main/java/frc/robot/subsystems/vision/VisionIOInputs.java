package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import org.littletonrobotics.junction.AutoLog;

@AutoLog
public class VisionIOInputs {
  public boolean cameraConnected;
  public boolean hasTarget;
  public double tx;
  public double ty;
  public double ta;

  public double fiducialID;
  //  public boolean hasAprilTag = false;
  public Pose2d robotPose = new Pose2d();
  //  public double aprilTagID = 0;
  public double latency;

  public boolean hasMegaTag2;
  public Pose2d megaTagPose; // can be either megatag 1 or 2
  public double timestamp;

  public int tagCount;
}
