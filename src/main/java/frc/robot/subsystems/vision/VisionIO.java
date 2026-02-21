package frc.robot.subsystems.vision;

public interface VisionIO {
  //   METHOD, not API
  // name doesn't matter here
  default void updateInputs(VisionIOInputs camOne, VisionIOInputs camTwo) {}
}
