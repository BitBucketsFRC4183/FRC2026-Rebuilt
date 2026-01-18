package frc.robot.subsystems.forearm;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkRelativeEncoder;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.constants.ForearmConstants;

public class ForearmIOSparkMax implements ForearmIO {

  private final SparkMax forearmMotor;
  private final SparkMax intakeMotor;

  private final SparkRelativeEncoder forearmEncoder;
  private final SparkClosedLoopController forearmClosedLoop;

  public ForearmIOSparkMax() {

    //Forearm

    forearmMotor =
            new SparkMax(ForearmConstants.FOREARM_MOTOR_CAN_ID, MotorType.kBrushless);

    SparkMaxConfig forearmConfig = new SparkMaxConfig();

    forearmConfig
            .inverted(ForearmConstants.FOREARM_MOTOR_INVERTED)
            .idleMode(SparkMaxConfig.IdleMode.kBrake);

    forearmConfig.encoder.positionConversionFactor(
            ForearmConstants.POSITION_CONVERSION_FACTOR);

    forearmConfig.closedLoop
            .pid(
                    ForearmConstants.kP,
                    ForearmConstants.kI,
                    ForearmConstants.kD)
            .outputRange(
                    ForearmConstants.MIN_OUTPUT,
                    ForearmConstants.MAX_OUTPUT);

    forearmMotor.configure(
            forearmConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters);

    forearmEncoder = (SparkRelativeEncoder) forearmMotor.getEncoder();
    forearmClosedLoop = forearmMotor.getClosedLoopController();

    //Intake

    intakeMotor =
            new SparkMax(ForearmConstants.INTAKE_MOTOR_CAN_ID, MotorType.kBrushless);

    SparkMaxConfig intakeConfig = new SparkMaxConfig();

    intakeConfig
            .inverted(ForearmConstants.INTAKE_MOTOR_INVERTED)
            .idleMode(SparkMaxConfig.IdleMode.kBrake);

    intakeMotor.configure(
            intakeConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters);
  }

  @Override
  public void updateInputs(ForearmIOInputs inputs) {
    inputs.forearmPositionDeg = forearmEncoder.getPosition();
    inputs.forearmAppliedOutput = forearmMotor.getAppliedOutput();
    inputs.intakeAppliedOutput = intakeMotor.getAppliedOutput();
  }

  //Forearm

  @Override
  public void setForearmPercent(double percent) {
    forearmMotor.set(percent);
  }

  @Override
  public void setForearmPosition(double degrees) {
    forearmClosedLoop.setSetpoint(degrees, SparkBase.ControlType.kPosition);
  }

  @Override
  public void stopForearm() {
    forearmMotor.stopMotor();
  }

  //Intake

  @Override
  public void setIntakePercent(double percent) {
    intakeMotor.set(percent);
  }

  @Override
  public void stopIntake() {
    intakeMotor.stopMotor();
  }
}
