package frc.robot.util;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

public class ActuatorIOSparkMax implements ActuatorIO {

  private final SparkMax actuatorMotor;

  public ActuatorIOSparkMax() {
    // Innit motor
    actuatorMotor =
        new SparkMax(ActuatorConstants.ACTUATOR_MOTOR_CAN_ID, SparkLowLevel.MotorType.kBrushless);
    SparkMaxConfig actuatorConfig = new SparkMaxConfig();
    actuatorConfig
        .inverted(ActuatorConstants.ACTUATOR_MOTOR_INVERTED)
        .idleMode(SparkMaxConfig.IdleMode.kBrake);
    actuatorMotor.configure(
        actuatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void updateInputs(ActuatorIOInputs inputs) {
    inputs.actuatorAppliedOutput = actuatorMotor.getAppliedOutput();
  }

  @Override
  public void setActuatorPercent(double percent) {}

  @Override
  public void stopActuator() {
    actuatorMotor.stopMotor();
  }
}
