package frc.robot.subsystems.intake;

public interface IntakeIO {

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
