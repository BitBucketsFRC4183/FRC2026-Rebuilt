package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.FeederConstants;

public class FeederSubsystem extends SubsystemBase {

    private final CANSparkMax motor;
    private final RelativeEncoder encoder;
    private final SparkMaxPIDController pid;

    public FeederSubsystem() {
        motor = new CANSparkMax(
                FeederConstants.FEEDER_MOTOR_ID,
                MotorType.kBrushless);

        motor.restoreFactoryDefaults();
        motor.setInverted(FeederConstants.MOTOR_INVERTED);
        motor.setIdleMode(FeederConstants.IDLE_MODE);
        motor.setSmartCurrentLimit(FeederConstants.CURRENT_LIMIT);

        encoder = motor.getEncoder();
        encoder.setPositionConversionFactor(
                FeederConstants.POSITION_CONVERSION);
        encoder.setVelocityConversionFactor(
                FeederConstants.VELOCITY_CONVERSION);

        pid = motor.getPIDController();
        pid.setP(FeederConstants.kP);
        pid.setI(FeederConstants.kI);
        pid.setD(FeederConstants.kD);
        pid.setFF(FeederConstants.kFF);

        // Software soft limits
        motor.enableSoftLimit(
                CANSparkMax.SoftLimitDirection.kForward, true);
        motor.enableSoftLimit(
                CANSparkMax.SoftLimitDirection.kReverse, true);

        motor.setSoftLimit(
                CANSparkMax.SoftLimitDirection.kForward,
                (float) FeederConstants.MAX_ANGLE);
        motor.setSoftLimit(
                CANSparkMax.SoftLimitDirection.kReverse,
                (float) FeederConstants.MIN_ANGLE);

        // Zero encoder at stowed position
        encoder.setPosition(0.0);
    }

    // Control

    /** Manual open-loop control */
    public void setPercentOutput(double output) {
        motor.set(
                Math.max(
                        -FeederConstants.MAX_OUTPUT,
                        Math.min(output, FeederConstants.MAX_OUTPUT)
                )
        );
    }


    public void setAngle(double radians) {
        pid.setReference(radians, CANSparkMax.ControlType.kPosition);
    }

    public void stop() {
        motor.stopMotor();
    }



    public double getAngle() {
        return encoder.getPosition();
    }

    public double getVelocity() {
        return encoder.getVelocity();
    }

    public boolean atAngle(double targetRadians, double toleranceRadians) {
        return Math.abs(getAngle() - targetRadians) <= toleranceRadians;
    }

    @Override
    public void periodic() {

    }
}
