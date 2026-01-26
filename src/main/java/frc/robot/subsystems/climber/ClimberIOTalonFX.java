package frc.robot.subsystems.climber;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.constants.ClimberConstants;

public class ClimberIOTalonFX implements ClimberIO {

    private final TalonFX armMotor;
    private final PositionDutyCycle armRequest = new PositionDutyCycle(0);

    public ClimberIOTalonFX() {
        armMotor = new TalonFX(ClimberConstants.ARM_MOTOR_CAN_ID);

        TalonFXConfiguration armConfig = new TalonFXConfiguration();

        // 1. Basic Motor behavior
        armConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        armConfig.MotorOutput.Inverted = ClimberConstants.ARM_MOTOR_INVERTED
                ? InvertedValue.Clockwise_Positive
                : InvertedValue.CounterClockwise_Positive;

        // 2. PID Gains for movement smoothness
        armConfig.Slot0.kP = ClimberConstants.ARM_kP;
        armConfig.Slot0.kD = ClimberConstants.ARM_kD;

        // 3. Safety: Current Limits (Prevents motor burnout)
        armConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        armConfig.CurrentLimits.SupplyCurrentLimit = 40.0; // Amps

        // 4. Safety: Soft Limits (Prevents physical over-travel)
        armConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        armConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
                (ClimberConstants.ARM_EXTENDED_DEG / 360.0) * ClimberConstants.ARM_GEAR_RATIO;

        armConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        armConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
                (ClimberConstants.ARM_RETRACTED_DEG / 360.0) * ClimberConstants.ARM_GEAR_RATIO;

        armMotor.getConfigurator().apply(armConfig);
    }

    @Override
    public void updateInputs(ClimberIOInputs inputs) {
        var posSignal = armMotor.getPosition();
        var velSignal = armMotor.getVelocity();
        var curSignal = armMotor.getSupplyCurrent();

        // Refresh signals for synchronized telemetry
        BaseStatusSignal.refreshAll(posSignal, velSignal, curSignal);

        inputs.armAngleDeg = (posSignal.getValueAsDouble() / ClimberConstants.ARM_GEAR_RATIO) * 360.0;
        inputs.armVelocityDegPerSec = (velSignal.getValueAsDouble() / ClimberConstants.ARM_GEAR_RATIO) * 360.0;
        inputs.armCurrentAmps = curSignal.getValueAsDouble();
    }

    // --- Movement Methods ---

    /** Moves the arm to the 'Up' position defined in Constants */
    public void moveUp() {
        setArmAngleDeg(ClimberConstants.ARM_EXTENDED_DEG);
    }

    /** Moves the arm to the 'Down' position defined in Constants */
    public void moveDown() {
        setArmAngleDeg(ClimberConstants.ARM_RETRACTED_DEG);
    }

    @Override
    public void setArmAngleDeg(double degrees) {
        double motorRotations = (degrees / 360.0) * ClimberConstants.ARM_GEAR_RATIO;
        armMotor.setControl(armRequest.withPosition(motorRotations));
    }

    @Override
    public void stopArm() {
        armMotor.stopMotor();
    }
}
