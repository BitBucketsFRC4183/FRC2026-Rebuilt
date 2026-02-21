package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
  @AutoLog
  public class ClimberIOInputs {
    public double climberHeight = 0.0;
    public double climberVoltage;
    public double climberCurrent;
  }

  public default void updateInputs(ClimberIOInputs inputs) {}

  public default void setTargetHeight(double height) {}

  public default void stopClimb() {}

  public default void setVoltage(double voltageSupplied) {}

  public default void setkG(double kG) {}
}
