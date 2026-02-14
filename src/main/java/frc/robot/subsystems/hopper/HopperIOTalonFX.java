package frc.robot.subsystems.hopper;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.constants.HopperConstants;

public class HopperIOTalonFX implements HopperIO {

    private final TalonFX conveyorMotor;
    private final DutyCycleOut percentRequest = new DutyCycleOut(0);

    public HopperIOTalonFX() {

        conveyorMotor = new TalonFX(HopperConstants.HOPPER_CONVEYOR_MOTOR_CAN_ID);

        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted =
                HopperConstants.HOPPER_CONVEYOR_MOTOR_INVERTED
                        ? InvertedValue.Clockwise_Positive
                        : InvertedValue.CounterClockwise_Positive;

        //Current Limits
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = 40.0;


        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = 80.0;

        conveyorMotor.getConfigurator().apply(config);
    }

    @Override
    public void updateInputs(HopperIOInputs inputs) {
        inputs.conveyorAppliedOutput =
                conveyorMotor.getDutyCycle().getValueAsDouble();
    }

    @Override
    public void setConveyorPercent(double percent) {
        conveyorMotor.setControl(percentRequest.withOutput(percent));
    }

    @Override
    public void stopConveyor() {
        conveyorMotor.stopMotor();
    }
}
