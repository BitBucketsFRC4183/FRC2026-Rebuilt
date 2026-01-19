package frc.robot.constants;


public final class HopperConstants {

    private HopperConstants() {}

    // CAN IDs
    public static final int HOPPER_CONVEYOR_MOTOR_CAN_ID = 9;
    public static final int HOPPER_OUTTAKE_LEFT_MOTOR_CAN_ID = 10;
    public static final int HOPPER_OUTTAKE_RIGHT_MOTOR_CAN_ID = 11;

    // Motor inversion
    public static final boolean HOPPER_CONVEYOR_MOTOR_INVERTED = false;
    public static final boolean HOPPER_OUTTAKE_LEFT_MOTOR_INVERTED = false;
    public static final boolean HOPPER_OUTTAKE_RIGHT_MOTOR_INVERTED = true;

    // Percent outputs
    public static final double CONVEYOR_FORWARD_PERCENT = 0.7;
    public static final double CONVEYOR_REVERSE_PERCENT = -0.7;

    public static final double OUTTAKE_FORWARD_PERCENT = 0.8;
    public static final double OUTTAKE_REVERSE_PERCENT = -0.8;
}
