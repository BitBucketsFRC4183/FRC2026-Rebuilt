package frc.robot.constants;

public final class IntakeConstants {

  private IntakeConstants() {}

  public static final int FOREARM_MOTOR_CAN_ID = 7;
  public static final boolean FOREARM_MOTOR_INVERTED = false;

  public static final double MIN_ANGLE_DEG = 0.0;
  public static final double MAX_ANGLE_DEG = 160.0;

  public static final double GEAR_RATIO = 100.0;
  public static final double DEGREES_PER_ROTATION = 360.0;

  public static final double POSITION_CONVERSION_FACTOR = DEGREES_PER_ROTATION / GEAR_RATIO;

  public static final double kP = 0.12;
  public static final double kI = 0.0;
  public static final double kD = 0.001;

  public static final double MAX_OUTPUT = 0.1;
  public static final double MIN_OUTPUT = -0.1;

  public static final double MANUAL_EXTEND_PERCENT = 0.4;
  public static final double MANUAL_RETRACT_PERCENT = -0.4;

  public static final int INTAKE_MOTOR_CAN_ID = 8;
  public static final boolean INTAKE_MOTOR_INVERTED = false;

  public static final double INTAKE_IN_PERCENT = 0.8;
  public static final double INTAKE_OUT_PERCENT = -0.8;
}
