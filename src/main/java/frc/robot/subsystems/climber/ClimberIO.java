package frc.robot.subsystems.climber;

public interface ClimberIO {

  public default void updateInputs(ClimberIOInputs inputs) {}

  public default void setTargetHeight(double height) {}

  public default void stopClimb() {}

  public default void setVoltage(double voltageSupplied) {}
}
