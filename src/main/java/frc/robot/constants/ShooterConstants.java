package frc.robot.constants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public final class ShooterConstants extends SubsystemBase {

  // Motor IDs
  public static final int flywheelID = 108;

  public static final boolean flywheelInverted = false;

  // In Inches por favor, to calculate tangential velocity
  public static final double radius = 1;

  // Arena Constants (Inches)
  public static final double hubHeight = 72 / 39.37;
  public static final double shooterHeight = 21.5 / 39.37;

  // Other Physics Stuff
  public static final double gravity = 9.81;
  public static final int maxRPM = 5000;

  // Assuming constant hood angle
  public static final double shooterAngle = 45;

  // Allowed Error for (actual distance away vs distance from launching)
  public static final double error = 5;

  //Allowed RPS offset before shooting
  public static final double tolerance = 5.0;

  // PID Constants
  public static final double kP = 0.00001;
  public static final double kI = 0;
  public static final double kD = 0;
  public static final double kMaxOutput = 1.0;
  public static final double kMinOutput = -1.0;

  // FF
  public static final double kS = 0;
  public static final double kV = 0;
  public static final double kA = 0;
  public static final double cosRatio = 0;

  // hood constants
  public static final double minHoodAngle = 10.0;
  public static final double maxHoodAngle = 80.0;
  // change accuracy later

}
