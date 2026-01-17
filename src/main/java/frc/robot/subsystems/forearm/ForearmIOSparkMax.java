package frc.robot.subsystems.forearm;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkRelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.constants.ForearmConstants;

public class ForearmIOSparkMax implements ForearmIO {

    private final SparkMax motor;
    private final SparkRelativeEncoder encoder;
    private final SparkClosedLoopController closedLoop;

    public ForearmIOSparkMax() {

        motor = new SparkMax(
                ForearmConstants.FOREARM_MOTOR_CAN_ID,
                MotorType.kBrushless
        );

        // Motor
        SparkMaxConfig config = new SparkMaxConfig();

        config
                .inverted(ForearmConstants.FOREARM_MOTOR_INVERTED)
                .idleMode(SparkMaxConfig.IdleMode.kBrake);

        // Position conversion
        config.encoder
                .positionConversionFactor(ForearmConstants.POSITION_CONVERSION_FACTOR);

        // PID
        config.closedLoop
                .pid(
                        ForearmConstants.kP,
                        ForearmConstants.kI,
                        ForearmConstants.kD
                )
                .outputRange(
                        ForearmConstants.MIN_OUTPUT,
                        ForearmConstants.MAX_OUTPUT
                );

        // Apply the config
        SparkFlexConfig config = new SparkFlexConfig();
        motor.configure(config);

        //Encoder
        encoder = (SparkRelativeEncoder) motor.getEncoder();
        closedLoop = motor.getClosedLoopController();
    }

    @Override
    public void updateInputs(ForearmIOInputs inputs) {
        inputs.positionDeg = encoder.getPosition();
        inputs.appliedOutput = motor.getAppliedOutput();
    }

    @Override
    public void setPercent(double percent) {
        motor.set(percent);
    }

    @Override
    public void setPosition(double degrees) {
        // Set the closed-loop target
        closedLoop.setSetpoint(
                degrees,
                com.revrobotics.spark.SparkBase.ControlType.kPosition
        );
    }

    @Override
    public void stop() {
        motor.stopMotor();
    }
}
