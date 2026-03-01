package frc.robot.constants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public final class ShooterConstants extends SubsystemBase {

  // Motor IDs
  public static final int flywheelID = 14;
  public static final int flywheelID2 = 15;
  public static final int intakeID = 31;

  // IntermediateMotorSpeed
  // O No
  public static final int intermediateSpeed = 20;

  public static final int flywheelDefaultSpeed = 48;
  public static final int maxRPS = 75;
  public static final boolean flywheelInverted = true;
  public static final boolean interInverted = true;

  // Allowed RPS offset before shooting
  // Not a perfect FF :sigh:
  public static final double tolerance = 0.5;

  // PID Constants for flywheel
  public static final double flywheel_kP = 0.2;
  public static final double flywheel_kI = 0.0;
  public static final double flywheel_kD = 0.0;

  // FF for flywheel
  public static final double flywheel_kS = 0.2;
  public static final double flywheel_kV = 0.11;
  public static final double flywheel_kA = 0.1;

  public static double statorCurrentLimit = 80;
  public static double supplyCurrentLimit = 50;
}
