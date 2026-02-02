package frc.robot.subsystems.shooter;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.*;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.constants.ShooterConstants;

public class ShooterIOSparkMax implements ShooterIO {

  private final SparkMax flywheel =
      new SparkMax(ShooterConstants.flywheelID, SparkLowLevel.MotorType.kBrushless);
  private final SparkMax intermediateMotors =
      new SparkMax(ShooterConstants.intermediateID, SparkLowLevel.MotorType.kBrushless);
  private final SparkRelativeEncoder flywheel_encoder =
      (SparkRelativeEncoder) flywheel.getEncoder();
  private final SparkRelativeEncoder intermediateMotors_encoder =
      (SparkRelativeEncoder) intermediateMotors.getEncoder();
  private final SparkClosedLoopController flywheel_controller = flywheel.getClosedLoopController();
  private final SparkClosedLoopController intermediateMotors_controller =
      intermediateMotors.getClosedLoopController();

  public ShooterIOSparkMax() {
    SparkMaxConfig config = new SparkMaxConfig();
    config.closedLoop.pid(ShooterConstants.kP, ShooterConstants.kI, ShooterConstants.kD);
    config.smartCurrentLimit(40);
    config.idleMode(SparkBaseConfig.IdleMode.kCoast);

    flywheel.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    intermediateMotors.configure(
        config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void setSpeed(double targetSpeed) {
    flywheel_controller.setSetpoint(targetSpeed * 60, SparkBase.ControlType.kVelocity);
  }

  @Override
  public void startIntermediateMotors() {
    intermediateMotors_controller.setSetpoint(
        ShooterConstants.intermediateSpeed, SparkBase.ControlType.kVelocity);
  }

  @Override
  public void stopMotor() {
    flywheel.stopMotor();
    intermediateMotors.stopMotor();
  }

  @Override
  public boolean speedReached(double targetSpeed) {
    return flywheel_encoder.getVelocity() == targetSpeed * 60;
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    //  If you close your eyes, this doesn't exist
  }
}
