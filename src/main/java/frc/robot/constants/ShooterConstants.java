package frc.robot.constants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public final class ShooterConstants extends SubsystemBase {

  // Motor IDs
  public static final int flywheelID = 14;
  public static final int flywheelID2 = 16;
  public static final int intakeID = 15;
  public static final int intermediateID = 13;

  // IntermediateMotorSpeed
  public static final int intermediateSpeed = 20;

  public static final int maxRPS = 75;

  public static final boolean flywheelInverted = true;
  public static final boolean intermediateInverted = true;

  // In Inches por favor, to calculate tangential velocity
  public static final double radius = 1 / 39.37;

  // Arena Constants (Inches)
  public static final double hubHeight = 72 / 39.37;
  public static final double shooterHeight = 21.5 / 39.37;

  // Assuming constant hood angle
  public static final double shooterAngle = 67;

  // Allowed RPS offset before shooting
  public static final double tolerance = 5.0;

  // PID Constants for flywheel
  public static final double flywheel_kP = 0.12;
  public static final double flywheel_kI = 0;
  public static final double flywheel_kD = 0;

  // FF for flywheel
  public static final double flywheel_kS = 0.2;
  public static final double flywheel_kV = 0.12;
  public static final double flywheel_kA = 0.1;

  // PID Constants for Intermediate Motor
  public static final double intermediate_kP = 0.12;
  public static final double intermediate_kI = 0;
  public static final double intermediate_kD = 0;

  // FF for Intermediate Motor
  public static final double intermediate_kS = 0.2;
  public static final double intermediate_kV = 0.12;
  public static final double intermediate_kA = 0.1;

  // hood constants lol
  public static final double minHoodAngle = 10.0;
  public static final double maxHoodAngle = 80.0;
  // change accuracy later

}
