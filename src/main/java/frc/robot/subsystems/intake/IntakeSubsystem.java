package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IntakeConstants;
import frc.robot.subsystems.vision.VisionMode;
import org.littletonrobotics.junction.Logger;

public class IntakeSubsystem extends SubsystemBase {

  public final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  private IntakeState currentState = IntakeState.DEPLOYED;

  private final SendableChooser<IntakeState> intakeStateChooser = new SendableChooser<>();

  public IntakeSubsystem(IntakeIO io) {
    this.io = io;
    intakeStateChooser.addOption("STOWED", IntakeState.STOWED);
    intakeStateChooser.addOption("DEPLOYED", IntakeState.DEPLOYED);
    intakeStateChooser.addOption("INTAKING", IntakeState.INTAKING);
    intakeStateChooser.addOption("OUTTAKING", IntakeState.OUTTAKING);

    SmartDashboard.putData("IntakeStateChooser", intakeStateChooser);
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

    IntakeState manualSelectMode = intakeStateChooser.getSelected();
    currentState = (manualSelectMode);

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
