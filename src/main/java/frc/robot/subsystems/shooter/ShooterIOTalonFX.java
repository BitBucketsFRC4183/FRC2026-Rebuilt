package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.constants.ShooterConstants;

public class ShooterIOTalonFX implements ShooterIO {
  public final TalonFX flywheel = new TalonFX(ShooterConstants.flywheelID);
  public final TalonFX intakeMotor = new TalonFX(ShooterConstants.intakeID);
  public final TalonFX intermediateMotor = new TalonFX(ShooterConstants.intermediateID);
  private final VelocityVoltage target = new VelocityVoltage(0);

  public ShooterIOTalonFX() {
    TalonFXConfiguration motorConfig = new TalonFXConfiguration();
    motorConfig.MotorOutput.Inverted =
        ShooterConstants.flywheelInverted
            ? com.ctre.phoenix6.signals.InvertedValue.Clockwise_Positive
            : com.ctre.phoenix6.signals.InvertedValue.CounterClockwise_Positive;

    // Hopefully can make the wind uptime for the flywheel faster
    motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    // PID and FF Configs
    Slot0Configs slot0 = motorConfig.Slot0;
    slot0.kP = ShooterConstants.flywheel_kP;
    slot0.kI = ShooterConstants.flywheel_kI;
    slot0.kD = ShooterConstants.flywheel_kD;
    slot0.kA = ShooterConstants.flywheel_kA;
    slot0.kV = ShooterConstants.flywheel_kV;
    slot0.kS = ShooterConstants.flywheel_kS;

    // Current Limits
    CurrentLimitsConfigs currentConfig = new CurrentLimitsConfigs();
    currentConfig.SupplyCurrentLimitEnable = true;
    currentConfig.SupplyCurrentLimit = 40;
    currentConfig.StatorCurrentLimitEnable = true;
    currentConfig.StatorCurrentLimit = 40;

    flywheel.getConfigurator().apply(motorConfig);
    flywheel.getConfigurator().apply(currentConfig);

    motorConfig.MotorOutput.Inverted =
            !ShooterConstants.flywheelInverted
                    ? com.ctre.phoenix6.signals.InvertedValue.Clockwise_Positive
                    : com.ctre.phoenix6.signals.InvertedValue.CounterClockwise_Positive;

    intakeMotor.getConfigurator().apply(motorConfig);
    intakeMotor.getConfigurator().apply(currentConfig);

    // Intermediate Motor Configs
    motorConfig.MotorOutput.Inverted =
        ShooterConstants.intermediateInverted
            ? com.ctre.phoenix6.signals.InvertedValue.Clockwise_Positive
            : com.ctre.phoenix6.signals.InvertedValue.CounterClockwise_Positive;

    slot0 = motorConfig.Slot0;
    slot0.kP = ShooterConstants.intermediate_kP;
    slot0.kI = ShooterConstants.intermediate_kI;
    slot0.kD = ShooterConstants.intermediate_kD;
    slot0.kA = ShooterConstants.intermediate_kA;
    slot0.kV = ShooterConstants.intermediate_kV;
    slot0.kS = ShooterConstants.intermediate_kS;

    motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    intermediateMotor.getConfigurator().apply(motorConfig);
    intermediateMotor.getConfigurator().apply(currentConfig);
  }

  @Override
  public void setSpeed(double targetSpeed) {
    flywheel.setControl(target.withVelocity(targetSpeed));
  }

  @Override
  public void startFeeding() {
    intakeMotor.setControl(target.withVelocity(ShooterConstants.intermediateSpeed));
    intermediateMotor.setControl(target.withVelocity(ShooterConstants.intermediateSpeed));
  }

  @Override
  public void stopMotor() {
    flywheel.stopMotor();
    intakeMotor.stopMotor();
    intermediateMotor.stopMotor();
  }

  @Override
  public boolean speedReached(double targetSpeed) {
    double currentTopFlywheelVelocity = flywheel.getVelocity().getValueAsDouble();
    return currentTopFlywheelVelocity < targetSpeed + ShooterConstants.tolerance
        && currentTopFlywheelVelocity > targetSpeed - ShooterConstants.tolerance;
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    inputs.flywheelCurrent = flywheel.getStatorCurrent().getValueAsDouble();
    inputs.flywheelVoltage = flywheel.getMotorVoltage().getValueAsDouble();
    inputs.flywheelCurrent2 = intakeMotor.getStatorCurrent().getValueAsDouble();
    inputs.flywheelVoltage2 = intakeMotor.getVelocity().getValueAsDouble();
    inputs.intermediateCurrent = intermediateMotor.getStatorCurrent().getValueAsDouble();
    inputs.intermediateVoltage = intermediateMotor.getMotorVoltage().getValueAsDouble();
  }
}

// Aidan's Social Security Number: 621 79 0241
