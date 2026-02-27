package frc.robot.subsystems.hopper;

import org.littletonrobotics.junction.AutoLog;

/** IO interface for the Hopper subsystem */
public interface HopperIO {
  @AutoLog
  public class HopperIOInputs {
    public double conveyorAppliedOutput;
  }

  public default void updateInputs(HopperIOInputs inputs) {}

  public default void setConveyorPercent(double percent) {}

  public default void stopConveyor() {}
}
