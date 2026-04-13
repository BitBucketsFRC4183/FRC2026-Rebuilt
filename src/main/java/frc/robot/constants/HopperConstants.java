package frc.robot.constants;

public final class HopperConstants {

  private HopperConstants() {}

  // CAN ID
  public static final int HOPPER_CONVEYOR_MOTOR_CAN_ID = 13;

  // Motor inversion
  public static final boolean MOTOR_INVERTED = true;

  // Percent outputs
  private static final double percent = 70;
  public static final double CONVEYOR_FORWARD_PERCENT = percent / 100;
  public static final double CONVEYOR_REVERSE_PERCENT = percent / 100;
}
