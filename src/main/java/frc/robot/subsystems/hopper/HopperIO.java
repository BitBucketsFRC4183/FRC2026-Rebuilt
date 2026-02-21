package frc.robot.subsystems.hopper;

/** IO interface for the Hopper subsystem */
public interface HopperIO {

  public default void updateInputs(HopperIOInputs inputs) {}

  public default void setConveyorPercent(double percent) {}

  public default void stopConveyor() {}
}
