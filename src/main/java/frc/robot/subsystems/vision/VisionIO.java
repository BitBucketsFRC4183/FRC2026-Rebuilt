package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.LimelightHelpers;
import org.littletonrobotics.junction.AutoLog;

public interface VisionIO {
    void updateInputs(VisionIOInputs inputs);
}
