package frc.robot.constants;

import edu.wpi.first.wpilibj.PneumaticsModuleType;

public final class IntakeConstants {

  // Motor
  public static final int INTAKE_MOTOR_ID = 31;
  public static final boolean MOTOR_INVERTED = false;

  public static final double INTAKE_SPEED = 1.0;
  public static final double OUTTAKE_SPEED = -0.8;
  public static final double HOLD_SPEED = 0.15;

  // Current limiting
  public static final double SUPPLY_CURRENT_LIMIT = 40.0;
  public static final double STATOR_CURRENT_LIMIT = 60.0;

  // Pneumatics
  public static final PneumaticsModuleType PNEUMATICS_TYPE = PneumaticsModuleType.REVPH;

  public static final int PISTON_FORWARD_CHANNEL = 0;
  public static final int PISTON_REVERSE_CHANNEL = 1;

  private IntakeConstants() {}
}
