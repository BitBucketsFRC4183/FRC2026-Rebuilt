package frc.robot.constants;

public class AimConstants {
  private AimConstants() {}

  public static final double KP = 0.6;
  public static final double KI = 0.0;
  public static final double KD = 0.0;

  public static final double TOLERANCE_DEG = 1.5;

  // ignored return velocity
  public static final double OUTPUT_DEADBAND = 0.05;

  // smallest minimum velocity
  public static final double MINIMUM_OUTPUT = 1;
}
