package frc.robot.subsystems.intake;

import static frc.robot.constants.IntakeConstants.*;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.revrobotics.servohub.ServoChannel;
import com.revrobotics.servohub.ServoChannel.ChannelId;
import com.revrobotics.servohub.ServoHub;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import frc.robot.constants.IntakeConstants;

public class IntakeIOTalonFX implements IntakeIO {

  private final TalonFX intakeMotor;
  private final DoubleSolenoid leftPiston;
  private final DoubleSolenoid rightPiston;
  private final ServoHub servoHub;
  private final ServoChannel servoChannel2;

  private double targetVelocityRPS;
  private final VelocityVoltage velocityRequest = new VelocityVoltage(0);
  private double lastServoAngleDegrees = 0.0;

  public IntakeIOTalonFX() {
    intakeMotor = new TalonFX(INTAKE_MOTOR_ID);
    TalonFXConfiguration motorConfig = new TalonFXConfiguration();

    MotorOutputConfigs outputConfigs = motorConfig.MotorOutput;
    outputConfigs.Inverted =
        MOTOR_INVERTED ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;

    Slot0Configs slot0 = motorConfig.Slot0;
    slot0.kP = kP;
    slot0.kI = kI;
    slot0.kD = kD;
    slot0.kA = kA;
    slot0.kV = kV;
    slot0.kS = kS;

    CurrentLimitsConfigs currentConfigs = motorConfig.CurrentLimits;
    currentConfigs.SupplyCurrentLimitEnable = true;
    currentConfigs.SupplyCurrentLimit = SUPPLY_CURRENT_LIMIT;
    currentConfigs.StatorCurrentLimitEnable = true;
    currentConfigs.StatorCurrentLimit = STATOR_CURRENT_LIMIT;

    intakeMotor.getConfigurator().apply(motorConfig);
    intakeMotor.stopMotor();

    leftPiston =
        new DoubleSolenoid(
            PNEUMATICS_HUB_CANID,
            PNEUMATICS_TYPE,
            LEFT_PISTON_FORWARD_CHANNEL,
            LEFT_PISTON_REVERSE_CHANNEL);

    rightPiston =
        new DoubleSolenoid(
            PNEUMATICS_HUB_CANID,
            PNEUMATICS_TYPE,
            RIGHT_PISTON_FORWARD_CHANNEL,
            RIGHT_PISTON_REVERSE_CHANNEL);

    servoHub = new ServoHub(IntakeConstants.hubCANID);
    servoChannel2 = servoHub.getServoChannel(ChannelId.kChannelId2);
    servoChannel2.setPowered(true);
    setServoAngle(0.0);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.motorVelocityRPS = intakeMotor.getVelocity().getValueAsDouble();
    inputs.motorVoltage = intakeMotor.getMotorVoltage().getValueAsDouble();
    inputs.motorCurrentAmps = intakeMotor.getSupplyCurrent().getValueAsDouble();
    inputs.motorTargetVelocityRPS = targetVelocityRPS;

    boolean extended =
        leftPiston.get() == DoubleSolenoid.Value.kForward
            && rightPiston.get() == DoubleSolenoid.Value.kForward;
    inputs.primaryPistonExtended = extended;
    inputs.secondaryPistonExtended = extended;

    inputs.servoAngleDegrees = lastServoAngleDegrees;
  }

  @Override
  public void setVelocity(double velocity) {
    targetVelocityRPS = velocity;
    intakeMotor.setControl(velocityRequest.withVelocity(velocity));
  }

  @Override
  public void stopMotor() {
    setVelocity(0.0);
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

  @Override
  public void setServoAngle(double angleDegrees) {
    lastServoAngleDegrees = angleDegrees;
    int pulseUs = angleToPulseWidth(angleDegrees);
    servoChannel2.setPulseWidth(pulseUs);
  }

  private int angleToPulseWidth(double angleDegrees) {
    return 500 + (int) (2000.0 * Math.min(Math.max(angleDegrees / 180.0, 0.0), 1.0));
  }
}
