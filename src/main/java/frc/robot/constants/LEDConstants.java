package frc.robot.constants;

public class LEDConstants {
  public static final int pwmChannel = 7;

  public enum LEDState {
    DISABLED(-1, BlinkinPattern.RAINBOW_RAINBOW_PALETTE),
    ALIGNED(1, BlinkinPattern.BPM_FOREST_PALETTE),
    INTAKING(5, BlinkinPattern.FIRE_LARGE),
    IDLE(9999, BlinkinPattern.TWINKLES_OCEAN_PALETTE);

    private final int priority;
    private final BlinkinPattern pattern;

    LEDState(int priority, BlinkinPattern pattern) {
      this.priority = priority;
      this.pattern = pattern;
    }

    public int getPriority() {
      return this.priority;
    }

    public BlinkinPattern getPattern() {
      return this.pattern;
    }
  }
}
