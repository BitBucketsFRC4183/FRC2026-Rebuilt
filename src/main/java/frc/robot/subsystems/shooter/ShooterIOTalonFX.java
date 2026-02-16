package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.constants.ShooterConstants;

public class ShooterIOTalonFX implements ShooterIO {
  public final TalonFX flywheelMotor = new TalonFX(ShooterConstants.flywheelID);
  public final TalonFX flywheelMotor2 = new TalonFX(ShooterConstants.flywheelID2);
  public final TalonFX intakeMotor = new TalonFX(ShooterConstants.intakeID);
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

    flywheelMotor.getConfigurator().apply(motorConfig);
    flywheelMotor.getConfigurator().apply(currentConfig);

    flywheelMotor2.getConfigurator().apply(motorConfig);
    flywheelMotor2.getConfigurator().apply(motorConfig);

    motorConfig.MotorOutput.Inverted =
            !ShooterConstants.flywheelInverted
                    ? com.ctre.phoenix6.signals.InvertedValue.Clockwise_Positive
                    : com.ctre.phoenix6.signals.InvertedValue.CounterClockwise_Positive;

    intakeMotor.getConfigurator().apply(motorConfig);
    intakeMotor.getConfigurator().apply(currentConfig);
  }

  @Override
  public void setSpeed(double targetSpeed) {
    flywheelMotor.setControl(target.withVelocity(targetSpeed));
  }

  @Override
  public void startFeeding() {
    intakeMotor.setControl(target.withVelocity(ShooterConstants.intermediateSpeed));
  }

  @Override
  public void stopMotor() {
    flywheelMotor.stopMotor();
    intakeMotor.stopMotor();
  }

  @Override
  public boolean speedReached(double targetSpeed) {
    double currentFlywheelMotorVelocity = flywheelMotor.getVelocity().getValueAsDouble();
    double currentFlywheelMotorVelocity2 = flywheelMotor2.getVelocity().getValueAsDouble();
    return currentFlywheelMotorVelocity < targetSpeed + ShooterConstants.tolerance
        && currentFlywheelMotorVelocity > targetSpeed - ShooterConstants.tolerance
            && currentFlywheelMotorVelocity2 < targetSpeed + ShooterConstants.tolerance
            && currentFlywheelMotorVelocity2 > targetSpeed - ShooterConstants.tolerance;
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    inputs.targetVelocity = ShooterSubsystem.getTargetVelocity();
    inputs.flywheelVelocity = flywheelMotor.getVelocity().getValueAsDouble();
    inputs.flywheelCurrent = flywheelMotor.getStatorCurrent().getValueAsDouble();
    inputs.flywheelVoltage = flywheelMotor.getMotorVoltage().getValueAsDouble();
    inputs.flywheelCurrent2 = intakeMotor.getStatorCurrent().getValueAsDouble();
    inputs.flywheelVoltage2 = intakeMotor.getMotorVoltage().getValueAsDouble();
    inputs.intakeCurrent = intakeMotor.getStatorCurrent().getValueAsDouble();
    inputs.intakeVoltage = intakeMotor.getMotorVoltage().getValueAsDouble();
  }
}

// Aidan's Social Security Number: 621 79 0241
