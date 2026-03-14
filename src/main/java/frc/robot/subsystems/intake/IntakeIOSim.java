package frc.robot.subsystems.intake;

public class IntakeIOSim implements IntakeIO {

  private double voltage;
  private boolean extended;
  private double servoAngle;

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.motorVoltage = voltage;
    inputs.motorCurrentAmps = 0.0;

    inputs.primaryPistonExtended = extended;
    inputs.secondaryPistonExtended = extended;

    inputs.servoAngleDegrees = servoAngle;
  }

  @Override
  public void setVoltage(double voltage) {
    this.voltage = voltage;
  }

  @Override
  public void stopMotor() {
    this.voltage = 0.0;
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
