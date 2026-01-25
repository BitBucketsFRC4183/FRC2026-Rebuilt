package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.constants.IntakeConstants;
import frc.robot.constants.ShooterConstants;

public class ShooterIOTalonFX implements ShooterIO {
    private final TalonFX finalFlywheel = new TalonFX(ShooterConstants.flywheelID);
    private final TalonFX intermediateMotor = new TalonFX(ShooterConstants.intermediateID);
    private final VelocityVoltage target = new VelocityVoltage(0);

    public ShooterIOTalonFX() {
        TalonFXConfiguration motorConfig = new TalonFXConfiguration();
        motorConfig.MotorOutput.Inverted = ShooterConstants.flywheelInverted
                ? com.ctre.phoenix6.signals.InvertedValue.Clockwise_Positive
                : com.ctre.phoenix6.signals.InvertedValue.CounterClockwise_Positive;

        //Hopefully can make the wind uptime for the flywheel faster
        motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        //PID and FF Configs
        Slot0Configs slot0 = motorConfig.Slot0;
        slot0.kP = ShooterConstants.kP;
        slot0.kI = ShooterConstants.kI;
        slot0.kD = ShooterConstants.kD;
        slot0.kA = ShooterConstants.kA;
        slot0.kV = ShooterConstants.kV;
        slot0.kS = ShooterConstants.kS;

        //Current Limits
        CurrentLimitsConfigs currentConfig = new CurrentLimitsConfigs();
        currentConfig.SupplyCurrentLimitEnable = true;
        currentConfig.SupplyCurrentLimit = 40;
        currentConfig.StatorCurrentLimitEnable = true;
        currentConfig.StatorCurrentLimit = 40;

        finalFlywheel.getConfigurator().apply(motorConfig);
        finalFlywheel.getConfigurator().apply(currentConfig);

        //Intermediate Motor Configs
        motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        intermediateMotor.getConfigurator().apply(motorConfig);
        intermediateMotor.getConfigurator().apply(currentConfig);
    }

    @Override
    public void setSpeed(double targetSpeed) {
        //Convert Radians / s to Rotations / s
        double targetRPS = targetSpeed / 2 / Math.PI;
        //Please be the same radius
        finalFlywheel.setControl(target.withVelocity(targetRPS));
    }

    @Override
    public void startIntermediateMotors() {
        intermediateMotor.setControl(target.withVelocity(ShooterConstants.intermediateSpeed));
    }

    @Override
    public void stopMotor() {
        finalFlywheel.stopMotor();
        intermediateMotor.stopMotor();
    }

    @Override
    public boolean speedReached(double targetSpeed) {
        double currentTopFlywheelVelocity =  finalFlywheel.getVelocity().getValueAsDouble();
        return currentTopFlywheelVelocity < targetSpeed + ShooterConstants.tolerance && currentTopFlywheelVelocity > targetSpeed - ShooterConstants.tolerance;
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        inputs.appliedFlywheelOutput = finalFlywheel.getDutyCycle().getValueAsDouble();
        inputs.appliedIntermediateOutput = intermediateMotor.getDutyCycle().getValueAsDouble();
        inputs.flywheelVelocity = finalFlywheel.getVelocity().getValueAsDouble();
        inputs.intermediateVelocity = intermediateMotor.getVelocity().getValueAsDouble();
    }
}

//Aidan's Social Security Number: 621 79 0241