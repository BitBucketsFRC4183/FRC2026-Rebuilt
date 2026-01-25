package frc.robot.subsystems.Arm;

import edu.wpi.first.math.controller.PIDController;
import org.littletonrobotics.junction.AutoLog;

public interface EndEffectorIO {
  void updateInputs(EndEffectorInputsAutoLogged inputs);

  @AutoLog
  public class EndEffectorInputs { // autolog volts
    public boolean isOpen = false;
    boolean endConnected = false;
    public double endMotorPositionRad = 0;
    public double endMotorVelocityRadPerSec = 0;
    public double endAppliedVolts = 0.0;
    public double endCurrentAmps = 0;
  }

  public default void setupPID(
      PIDController pid,
      double errorTolerance,
      double errorDerivativeTolerance,
      double minimumIntegral,
      double maximumIntegral) {
    pid.setTolerance(errorTolerance, errorDerivativeTolerance);
    pid.setIntegratorRange(minimumIntegral, maximumIntegral);
  }

  public default void setIsOpen(boolean setting) {}

  public default void updateInputs(EndEffectorInputs inputs) {}

  public default void disable() {}

  public default void setEndVelocity(double velocity) {}

  public default void setEndVoltage(double volts) {}
}
