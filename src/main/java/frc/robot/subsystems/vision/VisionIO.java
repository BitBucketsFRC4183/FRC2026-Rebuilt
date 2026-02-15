package frc.robot.subsystems.vision;

import java.util.Map;

public interface VisionIO {
  //   METHOD, not API
  void updateInputs(Map<String, VisionIOInputsAutoLogged> Inputs);
}
