package frc.robot.constants;

public final class ClimberConstants {

  private ClimberConstants() {}

  // CAN IDs
  public static final int ARM_MOTOR_CAN_ID = 30; // Kraken X60// Kraken X44

  // Inversion
  public static final boolean ARM_MOTOR_INVERTED = false;

  // Gear ratios (MEASURE THESE)
  public static final double ARM_GEAR_RATIO = 3.0; // motor rotations per arm rotation
  public static final double motorRadius = 2.0;

  // PID (starting values)
  public static final double ARM_kP = 1.0;
  public static final double ARM_kD = 2.0;
  public static final double ARM_kI = 0.5;

  // FF (starting values)
  public static final double ARM_kA = 1.0;
  public static final double ARM_kS = 2.0;
  public static final double ARM_kV = 2.0;

  // target values
  public static final double rung1Position = 5.0;
}
