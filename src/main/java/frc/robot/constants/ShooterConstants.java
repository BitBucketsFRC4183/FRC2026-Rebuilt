package frc.robot.constants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public final class ShooterConstants extends SubsystemBase {

  // Motor IDs
  public static final int flywheelID = 14;
  public static final int flywheelID2 = 15;
  public static final int intakeID = 16;

  // IntermediateMotorSpeed
  public static final int intermediateSpeed = 20;

  public static final int flywheelSpeed = 25;

  public static final int maxRPS = 75;

  public static final boolean flywheelInverted = true;
  public static final boolean intermediateInverted = true;

  // In Inches por favor, to calculate tangential velocity
  public static final double radius = 1 / 39.37;
  public static final double multiplier = 0.9;

  // Arena Constants (Inches)
  public static final double hubHeight = 72 / 39.37;
  public static final double shooterHeight = 21.5 / 39.37;

  // Assuming constant hood angle
  public static final double shooterAngle = 80.0;

  // Allowed RPS offset before shooting
  public static final double tolerance = 2.0;

  // PID Constants for flywheel
  public static final double flywheel_kP = 0.2;
  public static final double flywheel_kI = 0.0;
  public static final double flywheel_kD = 0.0;

  // FF for flywheel
  public static final double flywheel_kS = 0.2;
  public static final double flywheel_kV = 0.11;
  public static final double flywheel_kA = 0.1;

  // hood constants lol
  public static final double minHoodAngle = 10.0;
  public static final double maxHoodAngle = 80.0;
  // change accuracy later

}
