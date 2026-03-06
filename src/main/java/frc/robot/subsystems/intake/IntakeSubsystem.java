package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IntakeConstants;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

public class IntakeSubsystem extends SubsystemBase {

  public final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  private IntakeState currentState = IntakeState.STOWED;

  private final SendableChooser<IntakeState> intakeStateChooser = new SendableChooser<>();
  private final SendableChooser<Boolean> overrideToggle = new SendableChooser<>();

  private final LoggedMechanism2d mech = new LoggedMechanism2d(2, 2);
  private final LoggedMechanismRoot2d root = mech.getRoot("IntakeRoot", 1, 1);
  private final LoggedMechanismLigament2d intakeBar =
      root.append(
          new LoggedMechanismLigament2d("Intake", 0.0, 0.0, 8.0, new Color8Bit(Color.kPurple)));

  public IntakeSubsystem(IntakeIO io) {
    this.io = io;

    intakeStateChooser.setDefaultOption("DEPLOYED", IntakeState.DEPLOYED);
    intakeStateChooser.addOption("STOWED", IntakeState.STOWED);
    intakeStateChooser.addOption("INTAKING", IntakeState.INTAKING);
    intakeStateChooser.addOption("OUTTAKING", IntakeState.OUTTAKING);
    intakeStateChooser.addOption("HOLD", IntakeState.HOLD);

    overrideToggle.setDefaultOption("Intake Chooser Override ON", false);
    overrideToggle.addOption("Intake Chooser Override Off", true);

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

    boolean extended = inputs.primaryPistonExtended;

    if (extended) {
      intakeBar.setLength(0.8);
    } else {
      intakeBar.setLength(0.0);
    }

    if (currentState == IntakeState.INTAKING) {
      intakeBar.setColor(new Color8Bit(Color.kGreen));
    } else if (currentState == IntakeState.OUTTAKING) {
      intakeBar.setColor(new Color8Bit(Color.kRed));
    } else {
      intakeBar.setColor(new Color8Bit(Color.kPurple));
    }

    Logger.processInputs("Intake", inputs);
    Logger.recordOutput("Mechanisms/Intake", mech);
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
