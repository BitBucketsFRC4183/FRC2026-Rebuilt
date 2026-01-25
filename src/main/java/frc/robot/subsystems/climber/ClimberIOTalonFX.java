package frc.robot.subsystems.climber;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.constants.ClimberConstants;

public class ClimberIOTalonFX implements ClimberIO {

    private final TalonFX armMotor;
    private final TalonFX hookMotor;

    private final PositionDutyCycle armRequest = new PositionDutyCycle(0);
    private final PositionDutyCycle hookRequest = new PositionDutyCycle(0);

    public ClimberIOTalonFX() {

        //Arm X60 motor
        armMotor = new TalonFX(ClimberConstants.ARM_MOTOR_CAN_ID);

        TalonFXConfiguration armConfig = new TalonFXConfiguration();
        armConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        armConfig.MotorOutput.Inverted =
                ClimberConstants.ARM_MOTOR_INVERTED
                        ? InvertedValue.Clockwise_Positive
                        : InvertedValue.CounterClockwise_Positive;

        armConfig.Slot0.kP = ClimberConstants.ARM_kP;
        armConfig.Slot0.kD = ClimberConstants.ARM_kD;

        armMotor.getConfigurator().apply(armConfig);

        //Hook Motor
        hookMotor = new TalonFX(ClimberConstants.HOOK_MOTOR_CAN_ID);

        TalonFXConfiguration hookConfig = new TalonFXConfiguration();
        hookConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        hookConfig.MotorOutput.Inverted =
                ClimberConstants.HOOK_MOTOR_INVERTED
                        ? InvertedValue.Clockwise_Positive
                        : InvertedValue.CounterClockwise_Positive;

        hookConfig.Slot0.kP = ClimberConstants.HOOK_kP;

        hookMotor.getConfigurator().apply(hookConfig);
    }

    @Override
    public void updateInputs(ClimberIOInputs inputs) {
        inputs.armAngleDeg =
                (armMotor.getPosition().getValueAsDouble() / ClimberConstants.ARM_GEAR_RATIO) * 360.0;

        inputs.hookPositionRotations =
                hookMotor.getPosition().getValueAsDouble() / ClimberConstants.HOOK_GEAR_RATIO;
    }

    //Arm

    @Override
    public void setArmAngleDeg(double degrees) {
        double motorRotations =
                (degrees / 360.0) * ClimberConstants.ARM_GEAR_RATIO;
        armMotor.setControl(armRequest.withPosition(motorRotations));
    }

    @Override
    public void stopArm() {
        armMotor.stopMotor();
    }

    //Hooks

    @Override
    public void setHookPositionRotations(double rotations) {
        hookMotor.setControl(
                hookRequest.withPosition(rotations * ClimberConstants.HOOK_GEAR_RATIO));
    }

    @Override
    public void stopHooks() {
        hookMotor.stopMotor();
    }
}
