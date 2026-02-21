package frc.robot.subsystems.vision;

import java.util.Map;

public interface VisionIO {
  //   METHOD, not API
  //name doesn't matter here
  default void updateInputs(VisionIOInputs camOne, VisionIOInputs camTwo){}
}
