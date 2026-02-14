package frc.robot.subsystems.intake;

public interface IntakeIO {

  void updateInputs(IntakeIOInputs inputs);

  /* Forearm */
  void setForearmPercent(double percent);

  void setForearmPosition(double degrees);

  void stopForearm();

  /* Intake */
  void setIntakePercent(double percent);

  void stopIntake();
}
