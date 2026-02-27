package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
  @AutoLog
  class ShooterIOInputs {
    public double targetFlywheelSpeed;

    public double flywheelVelocity;
    public double flywheelVelocity2;

    public double flywheelPosition;
    public double flywheelPosition2;

    public double flywheelVoltage;
    public double flywheelVoltage2;

    public double flywheelCurrent;
    public double flywheelCurrent2;

    public double interVoltage;
    public double interCurrent;
  }

  public default void setFlywheelVoltage(double voltage) {}

  public default void setFlywheelSpeed(double targetSpeed) {}

  public default void startFeeding() {}

  public default void stopMotor() {}

  public default void updateInputs(ShooterIOInputs inputs) {}
}
