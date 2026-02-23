package frc.robot.subsystems.intake;

import static frc.robot.constants.IntakeConstants.PNEUMATICS_HUB_CANID;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import frc.robot.constants.IntakeConstants;

public class IntakeIOTalonFX implements IntakeIO {

  private final TalonFX intakeMotor;

  private final DoubleSolenoid leftPiston;
  private final DoubleSolenoid rightPiston;

  private final VelocityVoltage velocityRequest = new VelocityVoltage(0.0);

  public IntakeIOTalonFX() {
    intakeMotor = new TalonFX(IntakeConstants.INTAKE_MOTOR_ID);

    // Motor inversion
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

    // PID & kV Feedforward
    Slot0Configs slot0 = new Slot0Configs();
    slot0.kP = IntakeConstants.kP;
    slot0.kI = IntakeConstants.kI;
    slot0.kD = IntakeConstants.kD;
    slot0.kV = IntakeConstants.kV;

    intakeMotor.getConfigurator().apply(outputConfigs);
    intakeMotor.getConfigurator().apply(currentConfigs);
    intakeMotor.getConfigurator().apply(slot0);

    // Pneumatics
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

    boolean extended =
            leftPiston.get() == DoubleSolenoid.Value.kForward
                    && rightPiston.get() == DoubleSolenoid.Value.kForward;

    inputs.primaryPistonExtended = extended;
    inputs.secondaryPistonExtended = extended;
  }

  @Override
  public void setVelocity(double rpm) {
    double rps = rpm / 60.0;
    intakeMotor.setControl(velocityRequest.withVelocity(rps));
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
