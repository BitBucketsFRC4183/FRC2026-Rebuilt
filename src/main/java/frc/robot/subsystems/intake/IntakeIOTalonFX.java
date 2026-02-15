package frc.robot.subsystems.intake;

import static frc.robot.constants.IntakeConstants.PNEUMATICS_HUB_CANID;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import frc.robot.constants.IntakeConstants;

public class IntakeIOTalonFX implements IntakeIO {

  public final TalonFX intakeMotor;

  // Pneumatics (always actuated together)
  private final DoubleSolenoid leftPiston;
  private final DoubleSolenoid rightPiston;

  private final DutyCycleOut dutyCycleRequest = new DutyCycleOut(0.0);

  public IntakeIOTalonFX() {
    intakeMotor = new TalonFX(IntakeConstants.INTAKE_MOTOR_ID);

    // Motor output config
    MotorOutputConfigs outputConfigs = new MotorOutputConfigs();
    outputConfigs.Inverted =
        IntakeConstants.MOTOR_INVERTED
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;

    // Current limits
    CurrentLimitsConfigs currentConfigs = new CurrentLimitsConfigs();
    currentConfigs.SupplyCurrentLimitEnable = true;
    currentConfigs.SupplyCurrentLimit = IntakeConstants.SUPPLY_CURRENT_LIMIT;
    currentConfigs.StatorCurrentLimitEnable = true;
    currentConfigs.StatorCurrentLimit = IntakeConstants.STATOR_CURRENT_LIMIT;

    intakeMotor.getConfigurator().apply(outputConfigs);
    intakeMotor.getConfigurator().apply(currentConfigs);

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
    inputs.motorVelocityRPM = intakeMotor.getVelocity().getValueAsDouble() * 60.0;
    inputs.motorCurrentAmps = intakeMotor.getSupplyCurrent().getValueAsDouble();

    // If either piston disagrees, treat as NOT extended (safe default)
    boolean extended =
        leftPiston.get() == DoubleSolenoid.Value.kForward
            && rightPiston.get() == DoubleSolenoid.Value.kForward;

    inputs.primaryPistonExtended = extended;
    inputs.secondaryPistonExtended = extended;
  }

  @Override
  public void setMotorOutput(double percent) {
    intakeMotor.setControl(dutyCycleRequest.withOutput(percent));
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
