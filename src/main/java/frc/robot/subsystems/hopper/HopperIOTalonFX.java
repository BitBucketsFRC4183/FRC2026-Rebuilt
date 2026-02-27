package frc.robot.subsystems.hopper;

import static frc.robot.constants.IntakeConstants.*;
import static frc.robot.constants.IntakeConstants.kS;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import frc.robot.constants.HopperConstants;

public class HopperIOTalonFX implements HopperIO {

  private final TalonFX hopperMotor;
  private final VelocityVoltage velocityRequest = new VelocityVoltage(0);
  private double targetVelocityRPS = 0.0;

  public HopperIOTalonFX() {
    hopperMotor = new TalonFX(HopperConstants.HOPPER_MOTOR_ID);

    TalonFXConfiguration config = new TalonFXConfiguration();

    MotorOutputConfigs output = config.MotorOutput;
    output.Inverted =
        HopperConstants.MOTOR_INVERTED
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;

    Slot0Configs slot0 = config.Slot0;
    slot0.kP = kP;
    slot0.kI = kI;
    slot0.kD = kD;
    slot0.kA = kA;
    slot0.kV = kV;
    slot0.kS = kS;

    CurrentLimitsConfigs current = config.CurrentLimits;
    current.SupplyCurrentLimitEnable = true;
    current.SupplyCurrentLimit = HopperConstants.SUPPLY_CURRENT_LIMIT;
    current.StatorCurrentLimitEnable = true;
    current.StatorCurrentLimit = HopperConstants.STATOR_CURRENT_LIMIT;

    hopperMotor.getConfigurator().apply(config);

    hopperMotor.stopMotor();
  }

  @Override
  public void updateInputs(HopperIOInputs inputs) {
    inputs.motorVelocityRPS = hopperMotor.getVelocity().getValueAsDouble();
    inputs.motorVoltage = hopperMotor.getMotorVoltage().getValueAsDouble();
    inputs.motorCurrentAmps = hopperMotor.getSupplyCurrent().getValueAsDouble();
    inputs.motorTargetVelocityRPS = targetVelocityRPS;
  }

  @Override
  public void setVelocity(double velocity) {
    targetVelocityRPS = velocity;
    hopperMotor.setControl(velocityRequest.withVelocity(velocity));
  }
}
