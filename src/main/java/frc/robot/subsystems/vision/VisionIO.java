package frc.robot.subsystems.vision;

public interface VisionIO {
  //   METHOD, not API
  default void updateInputs(VisionIOInputs frontCamInputs, VisionIOInputs backCamInputs) {}
}
