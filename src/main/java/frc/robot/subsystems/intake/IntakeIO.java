package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {

  @AutoLog
  class IntakeIOInputs {
    public double motorVelocityRPM = 0.0;
    public double motorCurrentAmps = 0.0;
    public boolean primaryPistonExtended = false;
    public boolean secondaryPistonExtended = false;
  }

  /** Updates all sensor inputs */
  default void updateInputs(IntakeIOInputs inputs) {}

  /** Sets intake velocity in RPM */
  default void setVelocity(double rpm) {}

  /** Stops the intake motor */
  default void stopMotor() {
    setVelocity(0.0);
  }

  /** Extends the intake piston */
  default void extend() {}

  /** Retracts the intake piston */
  default void retract() {}
}
