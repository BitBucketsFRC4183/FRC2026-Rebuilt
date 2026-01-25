package frc.robot.subsystems.Arm;

import org.littletonrobotics.junction.AutoLog;

public interface EndEffectorEncoderIO {
  @AutoLog
  class EndEffectorEncoderIOInputs {
    public double velocity;
    public double position;
    boolean encoderConnected = false;
  }

  public default void updateInputs(EndEffectorEncoderIOInputs inputs) {}

  public default double getVelocity() {
    return 0.0;
  } // code encoder later

  public default double getDistance() {
    return 0.0;
  }

  public default boolean getStopped() {
    return true;
  }
}
