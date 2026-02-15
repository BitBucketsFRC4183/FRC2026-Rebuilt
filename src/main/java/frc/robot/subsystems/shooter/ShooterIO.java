package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
  @AutoLog
  public class ShooterIOInputs {
    public double flywheelVoltage;
    public double flywheelVoltage2;
    public double intermediateVoltage;
    public double flywheelCurrent;
    public double flywheelCurrent2;
    public double intermediateCurrent;
  }

  void setSpeed(double targetSpeed);

  void startIntermediateMotors();

  void stopMotor();

  boolean speedReached(double targetSpeed);

  void updateInputs(ShooterIOInputs inputs);
}
