package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;

public interface VisionIO {
    //
    double hasTarget();
    double tx();
    double ty();
    double ta();


    double botpose();
    Pose2d getRobotPose();

}
