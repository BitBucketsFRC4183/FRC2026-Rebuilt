package frc.robot.subsystems.climber;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.constants.ClimberConstants;
public class ClimberIOTalonFX implements ClimberIO {
private final TalonFX armMotor;
private final PositionDutyCycle armRequest = new PositionDutyCycle(0);
public ClimberIOTalonFX(){
    armMotor = new TalonFX(ClimberConstants.ARM_MOTOR_CAN_ID);
    TalonFXConfiguration armConfig = new TalonFXConfiguration();
    armConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    armConfig.MotorOutput.Inverted = ClimberConstants.ARM_MOTOR_INVERTED;
    ? InvertedValue.Clockwise_Positive
    :InvertedValue.CounterClockwise_Positive;
}
}
