package frc.robot.constants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public final class ShooterConstants extends SubsystemBase {

  // Motor IDs
  public static final int flywheelID = 108;
  public static final int intermediateID = 52;

  // IntermediateMotorSpeed
  public static final int intermediateSpeed = 20;

  public static final int maxRPS = 60;

  public static final boolean flywheelInverted = false;

  // In Inches por favor, to calculate tangential velocity
  public static final double radius = 1 / 39.37;

  // Arena Constants (Inches)
  public static final double hubHeight = 72 / 39.37;
  public static final double shooterHeight = 21.5 / 39.37;

  // Assuming constant hood angle
  public static final double shooterAngle = 67;

  // Allowed RPS offset before shooting
  public static final double tolerance = 5.0;

  // PID Constants
  public static final double kP = 0.0025;
  public static final double kI = 0;
  public static final double kD = 0;

  // FF
  public static final double kS = 0;
  public static final double kV = 0;
  public static final double kA = 0.1;
  public static final double cosRatio = 0;

  // hood constants
  public static final double minHoodAngle = 10.0;
  public static final double maxHoodAngle = 80.0;
  // change accuracy later

}
