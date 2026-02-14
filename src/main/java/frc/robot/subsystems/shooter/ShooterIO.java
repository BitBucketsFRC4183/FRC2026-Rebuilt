package frc.robot.subsystems.shooter;

public interface ShooterIO {
  void setSpeed(double targetSpeed);

  void startIntermediateMotors();

  void stopMotor();

  boolean speedReached(double targetSpeed);

  void updateInputs(ShooterIOInputs inputs);
}
