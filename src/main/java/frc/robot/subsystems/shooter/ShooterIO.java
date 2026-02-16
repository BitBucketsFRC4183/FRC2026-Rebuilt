package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
  @AutoLog
  class ShooterIOInputs {
    public double flywheelVoltage;
    public double flywheelVoltage2;
    public double intakeVoltage;
    public double flywheelCurrent;
    public double flywheelCurrent2;
    public double intakeCurrent;
  }

  void setSpeed(double targetSpeed);

  void startFeeding();

  void stopMotor();

  boolean speedReached(double targetSpeed);

  void updateInputs(ShooterIOInputs inputs);
}
