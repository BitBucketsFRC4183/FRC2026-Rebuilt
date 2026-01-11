package frc.robot.constants;

import com.revrobotics.CANSparkMax.IdleMode;

public final class FeederConstants {

    // CAN ID
    public static final int FEEDER_MOTOR_ID = 15;

    // Motor configuration
    public static final boolean MOTOR_INVERTED = false;
    public static final IdleMode IDLE_MODE = IdleMode.kBrake;
    public static final int CURRENT_LIMIT = 40;

    // Encoder / gearing
    // NEO internal encoder: 42 counts per motor revolution
    public static final double GEAR_RATIO = 100.0; // update to match your mechanism

    // Unit conversions
    public static final double POSITION_CONVERSION =
            2.0 * Math.PI / GEAR_RATIO; // radians
    public static final double VELOCITY_CONVERSION =
            POSITION_CONVERSION / 60.0; // rad/sec

    // Soft limits (radians)
    public static final double MIN_ANGLE = Math.toRadians(-10.0);
    public static final double MAX_ANGLE = Math.toRadians(115.0);

    // Manual control
    public static final double MAX_OUTPUT = 0.6;

    // PID values (safe starting point)
    public static final double kP = 3.0;
    public static final double kI = 0.0;
    public static final double kD = 0.2;
    public static final double kFF = 0.0;

    private FeederConstants() {}
}
