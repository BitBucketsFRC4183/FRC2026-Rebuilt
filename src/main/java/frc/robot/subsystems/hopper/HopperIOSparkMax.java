package frc.robot.subsystems.hopper;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import frc.robot.constants.HopperConstants;

public class HopperIOSparkMax implements HopperIO {

    private final SparkMax conveyorMotor;
    private final SparkMax outtakeLeftMotor;
    private final SparkMax outtakeRightMotor;

    public HopperIOSparkMax() {
        // Conveyor motor
        conveyorMotor = new SparkMax(HopperConstants.HOPPER_CONVEYOR_MOTOR_CAN_ID, SparkLowLevel.MotorType.kBrushless);
        SparkMaxConfig conveyorConfig = new SparkMaxConfig();
        conveyorConfig
                .inverted(HopperConstants.HOPPER_CONVEYOR_MOTOR_INVERTED)
                .idleMode(SparkMaxConfig.IdleMode.kBrake);
        conveyorMotor.configure(conveyorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Left outtake motor
        outtakeLeftMotor = new SparkMax(HopperConstants.HOPPER_OUTTAKE_LEFT_MOTOR_CAN_ID, SparkLowLevel.MotorType.kBrushless);
        SparkMaxConfig leftConfig = new SparkMaxConfig();
        leftConfig
                .inverted(HopperConstants.HOPPER_OUTTAKE_LEFT_MOTOR_INVERTED)
                .idleMode(SparkMaxConfig.IdleMode.kBrake);
        outtakeLeftMotor.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Right outtake motor
        outtakeRightMotor = new SparkMax(HopperConstants.HOPPER_OUTTAKE_RIGHT_MOTOR_CAN_ID, MotorType.kBrushless);
        SparkMaxConfig rightConfig = new SparkMaxConfig();
        rightConfig
                .inverted(HopperConstants.HOPPER_OUTTAKE_RIGHT_MOTOR_INVERTED)
                .idleMode(SparkMaxConfig.IdleMode.kBrake);
        outtakeRightMotor.configure(rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void updateInputs(HopperIOInputs inputs) {
        inputs.conveyorAppliedOutput = conveyorMotor.getAppliedOutput();
        inputs.outtakeLeftAppliedOutput = outtakeLeftMotor.getAppliedOutput();
        inputs.outtakeRightAppliedOutput = outtakeRightMotor.getAppliedOutput();
    }

    //Conveyor
    @Override
    public void setConveyorPercent(double percent) {
        conveyorMotor.set(percent);
    }

    @Override
    public void stopConveyor() {
        conveyorMotor.stopMotor();
    }

    //Outtake
    @Override
    public void setOuttakeLeftPercent(double percent) {
        outtakeLeftMotor.set(percent);
    }

    @Override
    public void setOuttakeRightPercent(double percent) {
        outtakeRightMotor.set(percent);
    }

    @Override
    public void stopOuttakes() {
        outtakeLeftMotor.stopMotor();
        outtakeRightMotor.stopMotor();
    }
}
