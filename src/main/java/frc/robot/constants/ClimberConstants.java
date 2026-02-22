package frc.robot.constants;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.units.Units;

public final class ClimberConstants {

  private ClimberConstants() {}

  // CAN IDs
  public static final int ARM_MOTOR_CAN_ID = 17; // Kraken X60// Kraken X44
  public static final CANBus climberBus = new CANBus("rio", "./logs/example.hoot");

  // Inversion
  public static final boolean ARM_MOTOR_INVERTED = false;

  // Gear ratios (MEASURE THESE)
  public static final double ARM_GEAR_RATIO = 135.0; // motor rotations per arm rotation
  public static final double spoolRadius = 0.5;

  // PID (starting values)
  public static final double ARM_kP = 2.0;
  public static final double ARM_kD = 2.0;
  public static final double ARM_kI = 3.0;

  // FF (starting values)
  public static final double ARM_kA = 1.0;
  public static final double ARM_kS = 2.0;
  public static final double ARM_kV = 1.0;
  public static final double ARM_kGUp = 1.0;
  public static final double ARM_kGDown = 100;
  public static final double speedConstant = 3;

  // target values
  public static final double rung1Position = Units.Inch.fromBaseUnits(27);
  public static final double rung2Position = Units.Inch.fromBaseUnits(18);

  // Limiters
  public static final double maxHeight = Units.Inch.fromBaseUnits(30);
  public static final double minHeight = Units.Inch.fromBaseUnits(0.05);
}
