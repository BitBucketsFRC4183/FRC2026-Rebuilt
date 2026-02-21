package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {

  @AutoLog
  public class IntakeIOInputs {
    public double motorVelocityRPM = 0.0;
    public double motorCurrentAmps = 0.0;
    public boolean primaryPistonExtended = false;
    public boolean secondaryPistonExtended = false;
  }
  /** Updates all sensor inputs */
  default void updateInputs(IntakeIOInputs inputs) {}

  /** Sets intake wheel motor output (-1 to 1) */
  default void setMotorOutput(double percent) {}

  /** Stops the intake motor */
  default void stopMotor() {
    setMotorOutput(0.0);
  }

  /** Extends the intake piston */
  default void extend() {}

  /** Retracts the intake piston */
  default void retract() {}
}
