package frc.robot.subsystems.Arm;

import edu.wpi.first.math.controller.PIDController;
import org.littletonrobotics.junction.AutoLog;

public interface EndEffectorIO {
    @AutoLog
    public class EndEffectorInputs {  //autolog volts
        public double gripperVolts = 0.0;
        public boolean isOpen = false;
    }

    public default void setupPID(PIDController pid, double errorTolerance, double errorDerivativeTolerance, double minimumIntegral, double maximumIntegral) {
        pid.setTolerance(errorTolerance, errorDerivativeTolerance);
        pid.setIntegratorRange(minimumIntegral, maximumIntegral);
    }
    public default void setIsOpen(boolean setting) {}

    public default void updateInputs(EndEffectorInputs inputs) {}

    public default void disable() {}

    public default void setEndVelocity(double velocity) {}

    public default void setEndVoltage(double volts) {}

}
