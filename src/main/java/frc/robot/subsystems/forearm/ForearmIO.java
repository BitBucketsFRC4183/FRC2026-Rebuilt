package frc.robot.subsystems.forearm;

public interface ForearmIO {

  void updateInputs(ForearmIOInputs inputs);

  /* Forearm */
  void setForearmPercent(double percent);
  void setForearmPosition(double degrees);
  void stopForearm();

  /* Intake */
  void setIntakePercent(double percent);
  void stopIntake();
}
