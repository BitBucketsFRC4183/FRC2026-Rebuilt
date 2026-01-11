package frc.robot.constants;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public final class FeederConstants {

    /* CAN */
    public static final int FEEDER_MOTOR_ID = 21;
    public static final String CAN_BUS = "rio"; // or "canivore"

    /* Motor Config */
    public static final InvertedValue MOTOR_INVERT =
            InvertedValue.CounterClockwise_Positive;
    public static final NeutralModeValue NEUTRAL_MODE =
            NeutralModeValue.Brake;

    /* Gear Ratio */
    // motor rotations per arm rotation
    public static final double GEAR_RATIO = 100.0;

    /* Arm Positions (arm rotations) */
    public static final double STOWED_POSITION = 0.0;
    public static final double INTAKE_POSITION = 0.35;
    public static final double SCORE_POSITION = 0.15;

    /* Soft Limits (arm rotations) */
    public static final double MIN_POSITION = -0.02;
    public static final double MAX_POSITION = 0.40;

    /* PID */
    public static final double kP = 35.0;
    public static final double kI = 0.0;
    public static final double kD = 0.5;
    public static final double kV = 0.0; // velocity feedforward
    public static final double kS = 0.0; // static feedforward
    public static final double kG = 0.0; // gravity feedforward (add later)

    /* Motion Limits */
    public static final double CRUISE_VELOCITY = 3.0; // arm rotations / sec
    public static final double ACCELERATION = 6.0;    // arm rotations / sec^2

    /* Safety */
    public static final double MAX_OUTPUT = 1.0;
    public static final double HOLD_DEADBAND = 0.01;

    private FeederConstants() {}
}
