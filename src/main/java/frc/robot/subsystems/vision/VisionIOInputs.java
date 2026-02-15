package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

@AutoLog
public class VisionIOInputs {
  public boolean cameraConnected;
  public boolean hasTarget;

  public double tx;
  public double ty;
  public double ta;

  public int aprilTagIDNumber;
  //  public boolean hasAprilTag = false;

  // remember, estimatedRobotPose origin from poseEstimator
  // vision use this data (robot orientation/robot pose), so the estimate will not too off
  // add vision measurement(vision pose, timestamp), then provide a new estimated robot pose
  public Pose2d estimatedRobotPose;
  public double latency;

//  public double poseAmbiguity;
  public boolean hasMegaTag2;

  // this is the estimated vision pose
  public Pose2d megaTagPose; // can be either megatag 1 or 2
  public double timestamp;

  public int tagCount;

  public Pose2d TargetHubPose2d;

  public double DistanceFromRobotToHub;

  public Rotation2d FieldAngleFromHubToRobot;

  public Rotation2d TurningAngle;
}
