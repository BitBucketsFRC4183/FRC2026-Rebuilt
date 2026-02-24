package frc.robot.subsystems.intake;

import static frc.robot.constants.IntakeConstants.PNEUMATICS_HUB_CANID;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import frc.robot.constants.IntakeConstants;

public class IntakeIOTalonFX implements IntakeIO {

  public final TalonFX intakeMotor;

  // Pneumatics (always actuated together)
  private final DoubleSolenoid leftPiston;
  private final DoubleSolenoid rightPiston;

  private final VelocityVoltage velocityRequest = new VelocityVoltage(0);

  public IntakeIOTalonFX() {
    intakeMotor = new TalonFX(IntakeConstants.INTAKE_MOTOR_ID);
    TalonFXConfiguration motorConfig = new TalonFXConfiguration();

    // Motor output config
    MotorOutputConfigs outputConfigs = motorConfig.MotorOutput;
    outputConfigs.Inverted =
        IntakeConstants.MOTOR_INVERTED
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;

    // PID and FF Configs
    Slot0Configs slot0 = motorConfig.Slot0;
    slot0.kP = IntakeConstants.kP;
    slot0.kI = IntakeConstants.kI;
    slot0.kD = IntakeConstants.kD;
    slot0.kA = IntakeConstants.kA;
    slot0.kV = IntakeConstants.kV;
    slot0.kS = IntakeConstants.kS;

    // Current limits
    CurrentLimitsConfigs currentConfigs = motorConfig.CurrentLimits;
    currentConfigs.SupplyCurrentLimitEnable = true;
    currentConfigs.SupplyCurrentLimit = IntakeConstants.SUPPLY_CURRENT_LIMIT;
    currentConfigs.StatorCurrentLimitEnable = true;
    currentConfigs.StatorCurrentLimit = IntakeConstants.STATOR_CURRENT_LIMIT;

    intakeMotor.getConfigurator().apply(motorConfig);

    // Pistons (mirrored / paired)
    leftPiston =
        new DoubleSolenoid(
            PNEUMATICS_HUB_CANID,
            IntakeConstants.PNEUMATICS_TYPE,
            IntakeConstants.LEFT_PISTON_FORWARD_CHANNEL,
            IntakeConstants.LEFT_PISTON_REVERSE_CHANNEL);

    rightPiston =
        new DoubleSolenoid(
            PNEUMATICS_HUB_CANID,
            IntakeConstants.PNEUMATICS_TYPE,
            IntakeConstants.RIGHT_PISTON_FORWARD_CHANNEL,
            IntakeConstants.RIGHT_PISTON_REVERSE_CHANNEL);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.motorVelocityRPS = intakeMotor.getVelocity().getValueAsDouble();
    inputs.motorCurrentAmps = intakeMotor.getSupplyCurrent().getValueAsDouble();

    // If either piston disagrees, treat as NOT extended (safe default)
    boolean extended =
        leftPiston.get() == DoubleSolenoid.Value.kForward
            && rightPiston.get() == DoubleSolenoid.Value.kForward;

    inputs.primaryPistonExtended = extended;
    inputs.secondaryPistonExtended = extended;
  }

  @Override
  public void setMotorOutput(double velocity) {
    intakeMotor.setControl(velocityRequest.withVelocity(velocity));
  }

  @Override
  public void extend() {
    leftPiston.set(DoubleSolenoid.Value.kForward);
    rightPiston.set(DoubleSolenoid.Value.kForward);
  }

  @Override
  public void retract() {
    leftPiston.set(DoubleSolenoid.Value.kReverse);
    rightPiston.set(DoubleSolenoid.Value.kReverse);
  }
}
