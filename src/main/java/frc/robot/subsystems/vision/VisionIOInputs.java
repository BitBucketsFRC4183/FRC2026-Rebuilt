package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import org.littletonrobotics.junction.AutoLog;

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

    public double latency = 0.0;
}
