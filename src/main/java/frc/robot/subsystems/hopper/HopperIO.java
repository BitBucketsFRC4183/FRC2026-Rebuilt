package frc.robot.subsystems.hopper;

import org.littletonrobotics.junction.AutoLog;

public interface HopperIO {

  @AutoLog
  public class HopperIOInputs {
    public double motorVelocityRPS = 0.0;
    public double motorTargetVelocityRPS = 0.0;
    public double motorVoltage = 0.0;
    public double motorCurrentAmps = 0.0;
  }

  default void updateInputs(HopperIOInputs inputs) {}

  default void setVelocity(double rps) {}

  default void stopMotor() {
    setVelocity(0.0);
  }
}
