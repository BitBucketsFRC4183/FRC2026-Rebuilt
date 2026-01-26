package frc.robot.subsystems.climber;

public interface ClimberIO {

  void updateInputs(ClimberIOInputs inputs);

  void setTargetHeight(double height);

  void stopClimb();
}
