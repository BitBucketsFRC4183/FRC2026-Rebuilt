package frc.robot.subsystems.led;

import frc.robot.constants.BlinkinPattern;
import org.littletonrobotics.junction.AutoLog;

public interface LEDIO {
  @AutoLog
  public class LEDInputs {
    public double speed;
    public double voltage;
    public boolean isConnected;
  }

  public default void updateInputs(LEDInputs inputs) {}
  ;

  public default void setPWMSpeed(double speed) {}
  ;

  public default void setPattern(BlinkinPattern pattern) {}
  ;
}
