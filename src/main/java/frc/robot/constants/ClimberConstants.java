package frc.robot.constants;

public final class ClimberConstants {

    private ClimberConstants() {}

    // CAN IDs
    public static final int ARM_MOTOR_CAN_ID = 30;   // Kraken X60
    public static final int HOOK_MOTOR_CAN_ID = 31;  // Kraken X44

    // Inversion
    public static final boolean ARM_MOTOR_INVERTED = false;
    public static final boolean HOOK_MOTOR_INVERTED = false;

    // Arm angles (degrees from ground)
    public static final double ARM_STOWED_DEG = 90.0;
    public static final double ARM_EXTENDED_DEG = 60.0;

    // Gear ratios (MEASURE THESE)
    public static final double ARM_GEAR_RATIO = 100.0;   // motor rotations per arm rotation
    public static final double HOOK_GEAR_RATIO = 20.0;

    // Motion limits (rotations)
    public static final double HOOK_MIN_ROTATIONS = 0.0;
    public static final double HOOK_MAX_ROTATIONS = 25.0;

    // PID (starting values)
    public static final double ARM_kP = 40.0;
    public static final double ARM_kD = 2.0;

    public static final double HOOK_kP = 15.0;
}
