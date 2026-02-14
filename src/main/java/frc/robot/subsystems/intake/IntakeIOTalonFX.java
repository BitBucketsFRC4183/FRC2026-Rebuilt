package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import frc.robot.constants.IntakeConstants;

public class IntakeIOTalonFX implements IntakeIO {

  private final TalonFX motor;
  private final DoubleSolenoid piston;

  private final DutyCycleOut dutyCycleRequest = new DutyCycleOut(0.0);

  public IntakeIOTalonFX() {
    motor = new TalonFX(IntakeConstants.INTAKE_MOTOR_ID);

    // Motor output config
    MotorOutputConfigs outputConfigs = new MotorOutputConfigs();
    outputConfigs.Inverted =
        IntakeConstants.MOTOR_INVERTED
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;

    // Current limits
    CurrentLimitsConfigs currentConfigs = new CurrentLimitsConfigs();
    currentConfigs.SupplyCurrentLimitEnable = true;
    currentConfigs.SupplyCurrentLimit = IntakeConstants.SUPPLY_CURRENT_LIMIT;
    currentConfigs.StatorCurrentLimitEnable = true;
    currentConfigs.StatorCurrentLimit = IntakeConstants.STATOR_CURRENT_LIMIT;

    motor.getConfigurator().apply(outputConfigs);
    motor.getConfigurator().apply(currentConfigs);

    piston =
        new DoubleSolenoid(
            IntakeConstants.PNEUMATICS_TYPE,
            IntakeConstants.PISTON_FORWARD_CHANNEL,
            IntakeConstants.PISTON_REVERSE_CHANNEL);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.motorVelocityRPM = motor.getVelocity().getValueAsDouble() * 60.0;
    inputs.motorCurrentAmps = motor.getSupplyCurrent().getValueAsDouble();
    inputs.pistonExtended = piston.get() == DoubleSolenoid.Value.kForward;
  }

  @Override
  public void setMotorOutput(double percent) {
    motor.setControl(dutyCycleRequest.withOutput(percent));
  }

  @Override
  public void extend() {
    piston.set(DoubleSolenoid.Value.kForward);
  }

  @Override
  public void retract() {
    piston.set(DoubleSolenoid.Value.kReverse);
  }
}
