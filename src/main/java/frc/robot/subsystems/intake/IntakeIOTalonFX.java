package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.constants.IntakeConstants;

public class IntakeIOTalonFX implements IntakeIO {

    private final TalonFX forearmMotor;
    private final TalonFX intakeMotor;

    private final PositionDutyCycle positionRequest = new PositionDutyCycle(0);
    private final DutyCycleOut percentRequest = new DutyCycleOut(0);

    public IntakeIOTalonFX() {

        // Forearm
        forearmMotor = new TalonFX(IntakeConstants.FOREARM_MOTOR_CAN_ID);

        TalonFXConfiguration intakeConfig = new TalonFXConfiguration();

        // Motor output
        intakeConfig.MotorOutput.Inverted =
                IntakeConstants.FOREARM_MOTOR_INVERTED
                        ? com.ctre.phoenix6.signals.InvertedValue.Clockwise_Positive
                        : com.ctre.phoenix6.signals.InvertedValue.CounterClockwise_Positive;

        intakeConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        // Sensor → degrees conversion
        intakeConfig.Feedback.SensorToMechanismRatio =
                IntakeConstants.POSITION_CONVERSION_FACTOR;

        // PID
        Slot0Configs slot0 = intakeConfig.Slot0;
        slot0.kP = IntakeConstants.kP;
        slot0.kI = IntakeConstants.kI;
        slot0.kD = IntakeConstants.kD;

        // Output limits
        intakeConfig.Voltage.PeakForwardVoltage = IntakeConstants.MAX_OUTPUT * 12.0;
        intakeConfig.Voltage.PeakReverseVoltage = IntakeConstants.MIN_OUTPUT * 12.0;

        forearmMotor.getConfigurator().apply(intakeConfig);

        /* Intake */
        intakeMotor = new TalonFX(IntakeConstants.INTAKE_MOTOR_CAN_ID);

        TalonFXConfiguration intakeConfig = new TalonFXConfiguration();

        intakeConfig.MotorOutput.Inverted =
                IntakeConstants.INTAKE_MOTOR_INVERTED
                        ? com.ctre.phoenix6.signals.InvertedValue.Clockwise_Positive
                        : com.ctre.phoenix6.signals.InvertedValue.CounterClockwise_Positive;

        intakeConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        intakeMotor.getConfigurator().apply(intakeConfig);
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.forearmPositionDeg =
                forearmMotor.getPosition().getValueAsDouble();

        inputs.forearmAppliedOutput =
                forearmMotor.getDutyCycle().getValueAsDouble();

        inputs.intakeAppliedOutput =
                intakeMotor.getDutyCycle().getValueAsDouble();
    }

    //Forearm

    @Override
    public void setForearmPercent(double percent) {
        forearmMotor.setControl(percentRequest.withOutput(percent));
    }

    @Override
    public void setForearmPosition(double degrees) {
        forearmMotor.setControl(positionRequest.withPosition(degrees));
    }

    @Override
    public void stopForearm() {
        forearmMotor.stopMotor();
    }

    // Intake

    @Override
    public void setIntakePercent(double percent) {
        intakeMotor.setControl(percentRequest.withOutput(percent));
    }

    @Override
    public void stopIntake() {
        intakeMotor.stopMotor();
    }
}
