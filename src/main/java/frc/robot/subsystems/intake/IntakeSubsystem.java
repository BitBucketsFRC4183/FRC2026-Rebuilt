package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {

  private final IntakeIO io;
  // private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  private IntakeState currentState = IntakeState.STOWED;

  public IntakeSubsystem(IntakeIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {

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
        io.setVelocity(IntakeConstants.INTAKE_RPM);
        break;

      case OUTTAKING:
        io.extend();
        io.setVelocity(IntakeConstants.OUTTAKE_RPM);
        break;
    }
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

}
