package frc.robot.subsystems.hopper;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Current;
import frc.robot.Robot;
import frc.robot.constants.HopperConstants;

public class HopperIOTalonFX implements HopperIO {

  public final TalonFX conveyorMotor;
  private final StatusSignal<Current> hopperCurrent;
  private final DutyCycleOut percentRequest = new DutyCycleOut(0);

  public HopperIOTalonFX() {

    conveyorMotor = new TalonFX(HopperConstants.HOPPER_CONVEYOR_MOTOR_CAN_ID);

    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted =
        HopperConstants.MOTOR_INVERTED
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;

    // Current Limits
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = 40.0;

    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = 80.0;

    conveyorMotor.getConfigurator().apply(config);

    this.hopperCurrent = conveyorMotor.getStatorCurrent();
    this.hopperCurrent.setUpdateFrequency(50);

    Robot.orchestra.addInstrument(conveyorMotor);
  }

  @Override
  public void updateInputs(HopperIOInputs inputs) {
    hopperCurrent.refresh();

    inputs.conveyorAppliedOutput = conveyorMotor.getDutyCycle().getValueAsDouble();
    inputs.conveyorVoltage = conveyorMotor.getMotorVoltage().getValueAsDouble();
    inputs.conveyorCurrent = hopperCurrent.getValueAsDouble();
    inputs.conveyorRPS = conveyorMotor.getVelocity().getValueAsDouble();
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
