package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {

  @AutoLog
  public class IntakeIOInputs {
    public double motorVelocityRPS = 0.0;
    public double motorTargetVelocityRPS = 0.0;
    public double motorVoltage = 0.0;
    public double motorCurrentAmps = 0.0;
    public boolean primaryPistonExtended = false;
    public boolean secondaryPistonExtended = false;
    public double servoAngleDegrees = 0.0;
  }

  default void updateInputs(IntakeIOInputs inputs) {}

  default void setVoltage(double volts) {}

  default void stopMotor() {
    setVoltage(0.0);
  }

  default void extend() {}

  default void retract() {}

  default void setServoAngle(double angleDegrees) {}

  default void setServoPulseWidth(int pulseWidth) {}
}
