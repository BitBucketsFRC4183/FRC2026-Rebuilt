package frc.robot.subsystems.led;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import frc.robot.constants.BlinkinPattern;
import frc.robot.constants.LEDConstants;

public class LEDIOBlinkin implements LEDIO {
  private final Spark blinkin;
  private final Debouncer connectionDebouncer = new Debouncer(0.5, Debouncer.DebounceType.kFalling);

  public LEDIOBlinkin() {
    this.blinkin = new Spark(LEDConstants.pwmChannel);
  }

  @Override
  public void updateInputs(LEDInputs inputs) {
    inputs.isConnected = connectionDebouncer.calculate(blinkin.isAlive());
    inputs.speed = blinkin.getVoltage() / 12;
    inputs.voltage = blinkin.getVoltage();
  }

  @Override
  public void setPWMSpeed(double speed) {
    blinkin.set(speed);
  }

  @Override
  public void setPattern(BlinkinPattern pattern) {
    setPWMSpeed(normalizePWMToSpeed(pattern.getPulseWidthMicros()));
  }

  private static double normalizePWMToSpeed(int value) {
    return ((value - 1005.0) / (1995.0 - 1005.0)) * 1.98 - 0.99;
  }
}
