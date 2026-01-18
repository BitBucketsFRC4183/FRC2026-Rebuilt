package frc.robot.subsystems.forearm;

public interface ForearmIO {
  void updateInputs(ForearmIOInputs inputs);

  void setPercent(double percent);

  void setPosition(double degrees);

  void stop();
}
