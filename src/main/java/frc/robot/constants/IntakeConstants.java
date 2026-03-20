package frc.robot.constants;

import edu.wpi.first.wpilibj.PneumaticsModuleType;

public final class IntakeConstants {

  // Motor
  public static final int INTAKE_MOTOR_ID = 16;
  public static final boolean MOTOR_INVERTED = false;

  // all in rps
  public static final double OUTTAKE_VOLTAGE = 8;
  public static final double STOWED_VOLTAGE = -7;
  public static final double INTAKE_VOLTAGE = -9;

  // Current limiting
  public static final double SUPPLY_CURRENT_LIMIT = 50.0;
  public static final double STATOR_CURRENT_LIMIT = 50.0;

  // Pneumatics
  public static final PneumaticsModuleType PNEUMATICS_TYPE = PneumaticsModuleType.REVPH;
  public static final int PNEUMATICS_HUB_CANID = 10;

  public static final int LEFT_PISTON_FORWARD_CHANNEL = 0;
  public static final int LEFT_PISTON_REVERSE_CHANNEL = 1;

  public static final int RIGHT_PISTON_FORWARD_CHANNEL = 2;
  public static final int RIGHT_PISTON_REVERSE_CHANNEL = 3;

  public static final int hubCANID = 12;
  public static final int INTAKE_SERVO_STOW_PULSEWIDTH = 1000;
  public static final int INTAKE_SERVO_DEPLOY_PULSEWIDTH = 1500;

  // Servo Stuff
  public static final double SERVO_0 = -20;
  public static final double SERVO_90 = 113;

  private IntakeConstants() {}
}
