package frc.robot.subsystems.forearm;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ForearmConstants;

public class ForearmSubsystem extends SubsystemBase {

  private final ForearmIO io;
  private final ForearmIOInputs inputs = new ForearmIOInputs();
  private double targetAngleDeg = 0.0;

  public ForearmSubsystem(ForearmIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);

    // Soft limits for forearm rotation
    if (inputs.forearmPositionDeg <= ForearmConstants.MIN_ANGLE_DEG
        && targetAngleDeg < inputs.forearmPositionDeg) {
      io.stopForearm();
    }

    if (inputs.forearmPositionDeg >= ForearmConstants.MAX_ANGLE_DEG
        && targetAngleDeg > inputs.forearmPositionDeg) {
      io.stopForearm();
    }
  }

  // ForearmConst

  /** Closed-loop position control */
  public void setAngle(double degrees) {
    targetAngleDeg =
        Math.max(ForearmConstants.MIN_ANGLE_DEG, Math.min(ForearmConstants.MAX_ANGLE_DEG, degrees));
    io.setForearmPosition(targetAngleDeg);
  }

  /** Manual percent output for forearm */
  public void runForearmManual(double percent) {
    io.setForearmPercent(percent);
  }

  /** Stop forearm motor */
  public void stopForearm() {
    io.stopForearm();
  }

  /** Current forearm angle */
  public double getAngle() {
    return inputs.forearmPositionDeg;
  }

  // Intake

  /** Run intake wheels */
  public void runIntake(double percent) {
    io.setIntakePercent(percent);
  }

  /** Stop intake wheels */
  public void stopIntake() {
    io.stopIntake();
  }

  /** Intake applied output (telemetry) */
  public double getIntakeOutput() {
    return inputs.intakeAppliedOutput;
  }
}
