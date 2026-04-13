package frc.robot.constants;

public class LEDConstants {
  public static final int pwmChannel = 7;

  public enum LEDState {
    DISABLED(-1, BlinkinPattern.RAINBOW_RAINBOW_PALETTE),
    ALIGNED(1, BlinkinPattern.STROBE_GOLD),
    SHOOTING(3, BlinkinPattern.COLOR_WAVES_PARTY_PALETTE),
    INTAKING(6, BlinkinPattern.FIRE_LARGE),
    PASSING(2, BlinkinPattern.STROBE_WHITE),
    IDLE(9999, BlinkinPattern.TWINKLES_OCEAN_PALETTE),
    INTAKE_STOWED(8, BlinkinPattern.STROBE_BLUE),
    INTAKE_DEPLOYED(7, BlinkinPattern.RAINBOW_FOREST_PALETTE);

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
