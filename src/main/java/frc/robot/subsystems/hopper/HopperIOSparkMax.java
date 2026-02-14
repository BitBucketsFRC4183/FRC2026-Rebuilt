package frc.robot.subsystems.hopper;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.constants.HopperConstants;

public class HopperIOSparkMax implements HopperIO {

  private final SparkMax conveyorMotor;

  public HopperIOSparkMax() {
    // Conveyor motor (NEO 550)
    conveyorMotor =
            new SparkMax(
                    HopperConstants.HOPPER_CONVEYOR_MOTOR_CAN_ID,
                    SparkLowLevel.MotorType.kBrushless);

    SparkMaxConfig conveyorConfig = new SparkMaxConfig();
    conveyorConfig
            .inverted(HopperConstants.HOPPER_CONVEYOR_MOTOR_INVERTED)
            .idleMode(SparkMaxConfig.IdleMode.kBrake);

    conveyorMotor.configure(
            conveyorConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters);
  }

  @Override
  public void updateInputs(HopperIOInputs inputs) {
    inputs.conveyorAppliedOutput = conveyorMotor.getAppliedOutput();
  }

  // Conveyor
  @Override
  public void setConveyorPercent(double percent) {
    conveyorMotor.set(percent);
  }

  @Override
  public void stopConveyor() {
    conveyorMotor.stopMotor();
  }
}
