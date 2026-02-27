package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IntakeConstants;
import org.littletonrobotics.junction.Logger;

public class IntakeSubsystem extends SubsystemBase {

  public final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  private IntakeState currentState = IntakeState.DEPLOYED;

  public IntakeSubsystem(IntakeIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);

    switch (currentState) {
      case STOWED:
        io.retract();
        io.stopMotor();
        break;

      case DEPLOYED:
        io.extend();
        io.stopMotor();
        break;

      case INTAKING:
        io.extend();
        io.setVelocity(IntakeConstants.INTAKE_SPEED);
        break;

      case OUTTAKING:
        io.extend();
        io.setVelocity(IntakeConstants.OUTTAKE_SPEED);
        break;

      case HOLD:
        io.extend();
        io.stopMotor();
        break;
    }

    Logger.processInputs("Intake", inputs);
    Logger.recordOutput("Intake/State", currentState.toString());
    Logger.recordOutput("Intake/IsRunning", isRunning());
    Logger.recordOutput(
            "Intake/Extended",
            inputs.primaryPistonExtended && inputs.secondaryPistonExtended);
    Logger.recordOutput(
            "Intake/VelocityError",
            inputs.motorTargetVelocityRPS - inputs.motorVelocityRPS);
    Logger.recordOutput("Intake/TargetRPS", inputs.motorTargetVelocityRPS);
    Logger.recordOutput("Intake/ActualRPS", inputs.motorVelocityRPS);
    Logger.recordOutput("Intake/Voltage", inputs.motorVoltage);
    Logger.recordOutput("Intake/Current", inputs.motorCurrentAmps);

    Logger.processInputs("Intake", inputs);
  }

  public boolean isRunning() {
    return currentState == IntakeState.INTAKING || currentState == IntakeState.OUTTAKING;
  }

  // State Control

  public void setState(IntakeState state) {
    this.currentState = state;
  }

  public IntakeState getState() {
    return currentState;
  }

  public void stow() {
    setState(IntakeState.STOWED);
  }

  public void deploy() {
    setState(IntakeState.DEPLOYED);
  }

  public void intake() {
    setState(IntakeState.INTAKING);
  }

  public void outtake() {
    setState(IntakeState.OUTTAKING);
  }

  public void hold() {
    setState(IntakeState.HOLD);
  }
}
