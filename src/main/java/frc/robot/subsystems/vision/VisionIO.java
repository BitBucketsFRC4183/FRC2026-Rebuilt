package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;

public interface VisionIO {
    //
    boolean hasTarget();
    double tx();
    double ty();
    double ta();

    boolean hasAprilTag();
    boolean botpose();
    Pose2d getRobotPose();

}
