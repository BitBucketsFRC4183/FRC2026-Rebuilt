package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
  @AutoLog
  class ShooterIOInputs {
    public double flywheelVelocity;
    public double flywheelVelocity2;
    public double flywheelVoltage;
    public double flywheelVoltage2;
    public double intakeVoltage;
    public double flywheelCurrent;
    public double flywheelCurrent2;
    public double intakeCurrent;
  }

  public default void setSpeed(double targetSpeed) {}

  public default void startFeeding() {}

  public default void stopMotor() {}

  public default void updateInputs(ShooterIOInputs inputs) {}
}
