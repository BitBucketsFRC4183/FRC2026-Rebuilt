package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.constants.ShooterConstants;

public class ShooterIOTalonFX implements ShooterIO {
  private final TalonFX finalFlywheel = new TalonFX(ShooterConstants.flywheelID);
  private final TalonFX finalFlywheel2 = new TalonFX(ShooterConstants.flywheelID2);
  private final TalonFX intermediateMotor = new TalonFX(ShooterConstants.intermediateID);
  private final VelocityVoltage target = new VelocityVoltage(0);

  public ShooterIOTalonFX() {
    Orchestra m_orchestra = new Orchestra();

    m_orchestra.addInstrument(finalFlywheel);
    m_orchestra.addInstrument(finalFlywheel2);

    m_orchestra.loadMusic("music/blackhole.chrp");
    m_orchestra.play();

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

    finalFlywheel.getConfigurator().apply(motorConfig);
    finalFlywheel.getConfigurator().apply(currentConfig);

    finalFlywheel2.getConfigurator().apply(motorConfig);
    finalFlywheel2.getConfigurator().apply(currentConfig);

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
    // Please be the same radius
    finalFlywheel.setControl(target.withVelocity(targetSpeed));
    finalFlywheel2.setControl(target.withVelocity(targetSpeed));
  }

  @Override
  public void startIntermediateMotors() {
    intermediateMotor.setControl(target.withVelocity(ShooterConstants.intermediateSpeed));
  }

  @Override
  public void stopMotor() {
    finalFlywheel.stopMotor();
    finalFlywheel2.stopMotor();
    intermediateMotor.stopMotor();
  }

  @Override
  public boolean speedReached(double targetSpeed) {
    double currentTopFlywheelVelocity = finalFlywheel.getVelocity().getValueAsDouble();
    double currentBottomFlywheelVelocity = finalFlywheel2.getVelocity().getValueAsDouble();
    return currentTopFlywheelVelocity < targetSpeed + ShooterConstants.tolerance
        && currentTopFlywheelVelocity > targetSpeed - ShooterConstants.tolerance
        && currentBottomFlywheelVelocity < targetSpeed + ShooterConstants.tolerance
        && currentBottomFlywheelVelocity > targetSpeed - ShooterConstants.tolerance;
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    inputs.flywheelCurrent = finalFlywheel.getStatorCurrent().getValueAsDouble();
    inputs.flywheelVoltage = finalFlywheel.getMotorVoltage().getValueAsDouble();
    inputs.flywheelCurrent2 = finalFlywheel2.getStatorCurrent().getValueAsDouble();
    inputs.flywheelVoltage2 = finalFlywheel2.getVelocity().getValueAsDouble();
    inputs.intermediateCurrent = intermediateMotor.getStatorCurrent().getValueAsDouble();
    inputs.intermediateVoltage = intermediateMotor.getMotorVoltage().getValueAsDouble();
  }
}

// Aidan's Social Security Number: 621 79 0241
