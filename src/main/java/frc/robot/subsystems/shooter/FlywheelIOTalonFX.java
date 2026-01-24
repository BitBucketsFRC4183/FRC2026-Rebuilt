package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.constants.IntakeConstants;
import frc.robot.constants.ShooterConstants;

public class FlywheelIOTalonFX implements FlywheelIO {
    private final TalonFX topFlywheel = new TalonFX(ShooterConstants.topFlywheelID);
    private final TalonFX bottomFlywheel = new TalonFX(ShooterConstants.bottomFlywheelID);
    private final VelocityVoltage target = new VelocityVoltage(0);

    public FlywheelIOTalonFX() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.Inverted = ShooterConstants.flywheelInverted
                ? com.ctre.phoenix6.signals.InvertedValue.Clockwise_Positive
                : com.ctre.phoenix6.signals.InvertedValue.CounterClockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        Slot0Configs slot0 = config.Slot0;
        slot0.kP = ShooterConstants.kP;
        slot0.kI = ShooterConstants.kI;
        slot0.kD = ShooterConstants.kD;
        slot0.kA = ShooterConstants.kA;
        slot0.kV = ShooterConstants.kV;
        slot0.kS = ShooterConstants.kS;

        topFlywheel.getConfigurator().apply(config);
    }
    @Override
    public void setSpeed(double targetSpeed) {
        double targetRPS = targetSpeed / 2 / Math.PI;
        topFlywheel.setControl(target.withVelocity(targetRPS));
        bottomFlywheel.setControl(target.withVelocity(targetRPS));
    }

    @Override
    public void stopMotor() {
        topFlywheel.stopMotor();
        bottomFlywheel.stopMotor();
    }
}
