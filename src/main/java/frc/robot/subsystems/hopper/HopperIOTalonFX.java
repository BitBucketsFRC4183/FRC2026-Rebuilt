package frc.robot.subsystems.hopper;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.constants.HopperConstants;

public class HopperIOTalonFX implements HopperIO {

    private final TalonFX conveyorMotor;
    private final TalonFX outtakeLeftMotor;
    private final TalonFX outtakeRightMotor;

    private final DutyCycleOut percentRequest = new DutyCycleOut(0);

    public HopperIOTalonFX() {

        // Conveyor
        conveyorMotor = new TalonFX(HopperConstants.HOPPER_CONVEYOR_MOTOR_CAN_ID);

        TalonFXConfiguration conveyorConfig = new TalonFXConfiguration();
        conveyorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        conveyorConfig.MotorOutput.Inverted =
                HopperConstants.HOPPER_CONVEYOR_MOTOR_INVERTED
                        ? InvertedValue.Clockwise_Positive
                        : InvertedValue.CounterClockwise_Positive;

        conveyorMotor.getConfigurator().apply(conveyorConfig);

        // Outtake Left
        outtakeLeftMotor =
                new TalonFX(HopperConstants.HOPPER_OUTTAKE_LEFT_MOTOR_CAN_ID);

        TalonFXConfiguration leftConfig = new TalonFXConfiguration();
        leftConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        leftConfig.MotorOutput.Inverted =
                HopperConstants.HOPPER_OUTTAKE_LEFT_MOTOR_INVERTED
                        ? InvertedValue.Clockwise_Positive
                        : InvertedValue.CounterClockwise_Positive;

        outtakeLeftMotor.getConfigurator().apply(leftConfig);

        // Outtake Right
        outtakeRightMotor =
                new TalonFX(HopperConstants.HOPPER_OUTTAKE_RIGHT_MOTOR_CAN_ID);

        TalonFXConfiguration rightConfig = new TalonFXConfiguration();
        rightConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        rightConfig.MotorOutput.Inverted =
                HopperConstants.HOPPER_OUTTAKE_RIGHT_MOTOR_INVERTED
                        ? InvertedValue.Clockwise_Positive
                        : InvertedValue.CounterClockwise_Positive;

        outtakeRightMotor.getConfigurator().apply(rightConfig);
    }

    @Override
    public void updateInputs(HopperIOInputs inputs) {
        inputs.conveyorAppliedOutput =
                conveyorMotor.getDutyCycle().getValueAsDouble();

        inputs.outtakeLeftAppliedOutput =
                outtakeLeftMotor.getDutyCycle().getValueAsDouble();

        inputs.outtakeRightAppliedOutput =
                outtakeRightMotor.getDutyCycle().getValueAsDouble();

    }

    // Conveyor

    @Override
    public void setConveyorPercent(double percent) {
        conveyorMotor.setControl(percentRequest.withOutput(percent));
    }

    @Override
    public void stopConveyor() {
        conveyorMotor.stopMotor();
    }

    // Outtake

    @Override
    public void setOuttakeLeftPercent(double percent) {
        outtakeLeftMotor.setControl(percentRequest.withOutput(percent));
    }

    @Override
    public void setOuttakeRightPercent(double percent) {
        outtakeRightMotor.setControl(percentRequest.withOutput(percent));
    }

    @Override
    public void stopOuttakes() {
        outtakeLeftMotor.stopMotor();
        outtakeRightMotor.stopMotor();
    }
}
