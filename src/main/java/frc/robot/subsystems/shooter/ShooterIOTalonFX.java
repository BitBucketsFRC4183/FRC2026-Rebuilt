package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.constants.ShooterConstants;

public class ShooterIOTalonFX implements ShooterIO {
    private final TalonFX flywheel = new TalonFX(ShooterConstants.flywheelID);
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

        flywheel.getConfigurator().apply(config);
    }

    @Override
    public void setSpeed(double targetSpeed) {
        //Convert Radians / s to Rotations / s
        double targetRPS = targetSpeed / 2 / Math.PI;
        //Please be the same radius
        flywheel.setControl(target.withVelocity(targetRPS));
    }

    @Override
    public void stopMotor() {
        flywheel.stopMotor();
    }

    @Override
    public boolean speedReached(double targetSpeed) {
        double currentTopFlywheelVelocity =  flywheel.getVelocity().getValueAsDouble();
        return currentTopFlywheelVelocity < targetSpeed + ShooterConstants.tolerance && currentTopFlywheelVelocity > targetSpeed - ShooterConstants.tolerance;
    }
}
