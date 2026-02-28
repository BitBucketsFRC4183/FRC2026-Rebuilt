package frc.robot.subsystems.intake;

public class IntakeIOSim implements IntakeIO {

  private double velocity;
  private boolean extended;
  private double servoAngle;

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.motorVelocityRPS = velocity;
    inputs.motorVoltage = 0.0;
    inputs.motorCurrentAmps = 0.0;
    inputs.motorTargetVelocityRPS = velocity;

    inputs.primaryPistonExtended = extended;
    inputs.secondaryPistonExtended = extended;

    inputs.servoAngleDegrees = servoAngle;
  }

  @Override
  public void setVelocity(double velocity) {
    this.velocity = velocity;
  }

  @Override
  public void stopMotor() {
    this.velocity = 0.0;
  }

  @Override
  public void extend() {
    extended = true;
  }

  @Override
  public void retract() {
    extended = false;
  }

  @Override
  public void setServoAngle(double angleDegrees) {
    servoAngle = angleDegrees;
  }
}
