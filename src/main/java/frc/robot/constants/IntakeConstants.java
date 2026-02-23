package frc.robot.constants;

import edu.wpi.first.wpilibj.PneumaticsModuleType;

public final class IntakeConstants {

  // Motor
  public static final int INTAKE_MOTOR_ID = 31;
  public static final boolean MOTOR_INVERTED = true;

  // Velocity targets (RPM)
  public static final double INTAKE_RPM = 5000.0;
  public static final double OUTTAKE_RPM = -4000.0;

  // PID & Feedforward control
  public static final double kP = 0.12;
  public static final double kI = 0.0;
  public static final double kD = 0.0;
  public static final double kV = 0.1;

  // Current limiting
  public static final double SUPPLY_CURRENT_LIMIT = 30.0;
  public static final double STATOR_CURRENT_LIMIT = 50.0;

  // Pneumatics
  public static final PneumaticsModuleType PNEUMATICS_TYPE = PneumaticsModuleType.REVPH;
  public static final int PNEUMATICS_HUB_CANID = 10;

  public static final int LEFT_PISTON_FORWARD_CHANNEL = 0;
  public static final int LEFT_PISTON_REVERSE_CHANNEL = 1;

  public static final int RIGHT_PISTON_FORWARD_CHANNEL = 2;
  public static final int RIGHT_PISTON_REVERSE_CHANNEL = 3;

  private IntakeConstants() {}
}
