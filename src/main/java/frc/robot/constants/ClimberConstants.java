package frc.robot.constants;

import com.ctre.phoenix6.CANBus;

public final class ClimberConstants {

  private ClimberConstants() {}

  // CAN IDs
  public static final int ARM_MOTOR_CAN_ID = 16; // Kraken X60// Kraken X44
  public static final CANBus climberBus = new CANBus("rio", "./logs/example.hoot");

  // Inversion
  public static final boolean ARM_MOTOR_INVERTED = false;

  // Gear ratios (MEASURE THESE)
  public static final double ARM_GEAR_RATIO = 45.0; // motor rotations per arm rotation
  public static final double spoolRadius = 0.375;

  // PID (starting values)
  public static final double ARM_kP = 2.0;
  public static final double ARM_kD = 2.0;
  public static final double ARM_kI = 3.0;

  // FF (starting values)
  public static final double ARM_kA = 1.0;
  public static final double ARM_kS = 2.0;
  public static final double ARM_kV = 0.5;

  // target values
  public static final double rung1Position = 5.0;

  //Limiters
  public static final double maxHeight = 5.0;
  public static final double minHeight = 1.0;
}
