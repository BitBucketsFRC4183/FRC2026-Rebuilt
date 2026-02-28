package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IntakeConstants;
import org.littletonrobotics.junction.Logger;

public class IntakeSubsystem extends SubsystemBase {

  public final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  private IntakeState currentState = IntakeState.DEPLOYED;

  private final SendableChooser<IntakeState> intakeStateChooser = new SendableChooser<>();
  private final SendableChooser<Boolean> overrideToggle = new SendableChooser<>();

  public IntakeSubsystem(IntakeIO io) {
    this.io = io;

    intakeStateChooser.setDefaultOption("DEPLOYED", IntakeState.DEPLOYED);
    intakeStateChooser.addOption("STOWED", IntakeState.STOWED);
    intakeStateChooser.addOption("INTAKING", IntakeState.INTAKING);
    intakeStateChooser.addOption("OUTTAKING", IntakeState.OUTTAKING);
    intakeStateChooser.addOption("HOLD", IntakeState.HOLD);

    overrideToggle.setDefaultOption("Override OFF", false);
    overrideToggle.addOption("Override ON", true);

    SmartDashboard.putData("IntakeStateChooser", intakeStateChooser);
    SmartDashboard.putData("IntakeManualOverride", overrideToggle);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);

    boolean overrideEnabled = overrideToggle.getSelected() != null && overrideToggle.getSelected();
    IntakeState selected = intakeStateChooser.getSelected();

    if (overrideEnabled && selected != null) {
      currentState = selected;
    }

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
  }

  public boolean isRunning() {
    return currentState == IntakeState.INTAKING || currentState == IntakeState.OUTTAKING;
  }

  public void setState(IntakeState state) {
    if (!(overrideToggle.getSelected() != null && overrideToggle.getSelected())) {
      this.currentState = state;
    }
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
