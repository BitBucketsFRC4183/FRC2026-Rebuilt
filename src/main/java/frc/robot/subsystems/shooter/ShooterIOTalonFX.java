package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.constants.ShooterConstants;
import org.littletonrobotics.junction.AutoLogOutput;

public class ShooterIOTalonFX implements ShooterIO {
    private final TalonFX topFlywheel = new TalonFX(ShooterConstants.topFlywheelID);
    private final TalonFX bottomFlywheel = new TalonFX(ShooterConstants.bottomFlywheelID);
    private final VelocityVoltage target = new VelocityVoltage(0);

    public ShooterIOTalonFX() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.Inverted = ShooterConstants.flywheelInverted
                ? com.ctre.phoenix6.signals.InvertedValue.Clockwise_Positive
                : com.ctre.phoenix6.signals.InvertedValue.CounterClockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        //PID and FF Configs
        Slot0Configs slot0 = config.Slot0;
        slot0.kP = ShooterConstants.kP;
        slot0.kI = ShooterConstants.kI;
        slot0.kD = ShooterConstants.kD;
        slot0.kA = ShooterConstants.kA;
        slot0.kV = ShooterConstants.kV;
        slot0.kS = ShooterConstants.kS;

        topFlywheel.getConfigurator().apply(config);

        config.MotorOutput.Inverted = !ShooterConstants.flywheelInverted
                ? com.ctre.phoenix6.signals.InvertedValue.Clockwise_Positive
                : com.ctre.phoenix6.signals.InvertedValue.CounterClockwise_Positive;

        bottomFlywheel.getConfigurator().apply(config);
    }

    @AutoLogOutput
    @Override
    public void setSpeed(double targetSpeed) {
        //Convert Radians / s to Rotations / s
        double targetRPS = targetSpeed / 2 / Math.PI;
        //Please be the same radius
        topFlywheel.setControl(target.withVelocity(targetRPS));
        bottomFlywheel.setControl(target.withVelocity(targetRPS));
    }

    @Override
    public void stopMotor() {
        topFlywheel.stopMotor();
        bottomFlywheel.stopMotor();
    }

    @Override
    public boolean speedReached(double targetSpeed) {
        double currentTopFlywheelVelocity =  topFlywheel.getVelocity().getValueAsDouble();
        double currentBottomFlywheelVelocity = bottomFlywheel.getVelocity().getValueAsDouble();
        return currentTopFlywheelVelocity < targetSpeed + ShooterConstants.tolerance
                && currentTopFlywheelVelocity > targetSpeed - ShooterConstants.tolerance
                && currentBottomFlywheelVelocity < targetSpeed + ShooterConstants.tolerance
                && currentBottomFlywheelVelocity > targetSpeed - ShooterConstants.tolerance;
    }
}
