package frc.robot.subsystems.hopper;

/** IO interface for the Hopper subsystem */
public interface HopperIO {

  void updateInputs(HopperIOInputs inputs);

  // Conveyor
  void setConveyorPercent(double percent);

  void stopConveyor();

  // Outtakemotors
  void setOuttakeLeftPercent(double percent);

  void setOuttakeRightPercent(double percent);

  void stopOuttakes();
}
