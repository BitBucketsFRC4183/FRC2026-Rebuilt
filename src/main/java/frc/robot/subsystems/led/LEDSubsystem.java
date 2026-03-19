package frc.robot.subsystems.led;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.LEDConstants.LEDState;
import frc.robot.subsystems.intake.IntakeState;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.vision.AutoAimUtil;
import java.util.Optional;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class LEDSubsystem extends SubsystemBase {
  private final Supplier<Pose2d> poseSupplier;
  private final IntakeSubsystem intake;
  private final LEDIO ledController;
  private LEDState state;
  private final LEDInputsAutoLogged inputs = new LEDInputsAutoLogged();

  public LEDSubsystem(LEDIO ledController, Supplier<Pose2d> poseSupplier, IntakeSubsystem intake) {
    this.poseSupplier = poseSupplier;
    this.intake = intake;
    this.ledController = ledController;
    this.state = LEDState.DISABLED;
  }

  @Override
  public void periodic() {
    ledController.updateInputs(inputs);
    Logger.processInputs("LED", inputs);

    state = getNewState().orElse(state);

    Logger.recordOutput("LED/LEDState", state);
    Logger.recordOutput("LED/Pattern", state.getPattern());

    ledController.setPattern(state.getPattern());
  }

  private Optional<LEDState> getNewState() {
    Optional<LEDState> newState = Optional.empty();

    for (LEDState candidateState : LEDState.values()) {
      if (newState.isPresent()) {
        if (candidateState.getPriority() > newState.get().getPriority()) {
          continue;
        }
      }

      switch (candidateState) {
        case DISABLED:
          if (DriverStation.isDisabled()) {
            newState = Optional.of(candidateState);
          }
          break;
        case ALIGNED:
          Pose2d currentPose = poseSupplier.get();
          Rotation2d currentRot = currentPose.getRotation();
          double tolerance_degs = 2;

          if ((AutoAimUtil.getAngletoHub(currentPose).getDegrees() + tolerance_degs
                  > currentRot.getDegrees())
              && (AutoAimUtil.getAngletoHub(currentPose).getDegrees() - tolerance_degs
                  < currentRot.getDegrees())) {
            newState = Optional.of(candidateState);
          }
          break;
        case INTAKING:
          if (intake.getState() == IntakeState.INTAKING) {
            newState = Optional.of(candidateState);
          }
          break;
      }
    }

    return newState;
  }
}
