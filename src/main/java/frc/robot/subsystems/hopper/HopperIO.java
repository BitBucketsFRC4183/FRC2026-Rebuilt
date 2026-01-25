package frc.robot.subsystems.hopper;

/** IO interface for the Hopper subsystem */
public interface HopperIO {

  void updateInputs(HopperIOInputs inputs);

  void setConveyorPercent(double percent);

  void stopConveyor();
}
