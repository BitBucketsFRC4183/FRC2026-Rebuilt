package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;

public interface VisionIO {
    //
    boolean hasTarget();
    boolean tx();
    boolean ty();
    boolean ta();


    boolean botpose();
    Pose2d getRobotPose();

}
