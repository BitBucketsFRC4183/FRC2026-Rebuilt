package frc.robot.subsystems.climber;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.constants.ClimberConstants;

public class ClimberIOTalonFX implements ClimberIO {

  private final TalonFX climbMotor;

  private final PositionVoltage climbRequest = new PositionVoltage(0);

  public ClimberIOTalonFX() {

    // Arm X60 motor
    climbMotor = new TalonFX(ClimberConstants.ARM_MOTOR_CAN_ID);

    TalonFXConfiguration climbConfig = new TalonFXConfiguration();
    climbConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    climbConfig.MotorOutput.Inverted =
        ClimberConstants.ARM_MOTOR_INVERTED
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;

    climbConfig.Slot0.kP = ClimberConstants.ARM_kP;
    climbConfig.Slot0.kD = ClimberConstants.ARM_kD;
    climbConfig.Slot0.kI = ClimberConstants.ARM_kI;

    climbConfig.Slot0.kA = ClimberConstants.ARM_kA;
    climbConfig.Slot0.kV = ClimberConstants.ARM_kV;
    climbConfig.Slot0.kS = ClimberConstants.ARM_kS;

    climbMotor.getConfigurator().apply(climbConfig);
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    inputs.climberHeight =
        (climbMotor.getPosition().getValueAsDouble() / ClimberConstants.ARM_GEAR_RATIO)
            * ClimberConstants.spoolRadius
            * 2
            * Math.PI;
    inputs.currentVoltage = climbMotor.getMotorVoltage().getValueAsDouble();
  }

  // Arm

  @Override
  public void setTargetHeight(double height) {
    double motorRotations =
        height / (2 * Math.PI * ClimberConstants.spoolRadius) * ClimberConstants.ARM_GEAR_RATIO;

    climbMotor.setControl(climbRequest.withPosition(motorRotations));
  }

  @Override
  public void stopClimb() {
    climbMotor.stopMotor();
  }

  @Override
  public double getCurrentHeight() {
    double climberHeight =
        (climbMotor.getPosition().getValueAsDouble() / ClimberConstants.ARM_GEAR_RATIO)
            * ClimberConstants.spoolRadius
            * 2
            * Math.PI;
    return climberHeight;
  }

  @Override
  public double getCurrentVoltage() {
    double currentVoltage = climbMotor.getMotorVoltage().getValueAsDouble();
    return currentVoltage;
  }
}
