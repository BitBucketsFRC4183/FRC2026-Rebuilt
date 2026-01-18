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

    // Soft limits: stop at min/max
    if (inputs.positionDeg <= ForearmConstants.MIN_ANGLE_DEG
        && targetAngleDeg < inputs.positionDeg) {
      io.stop();
    }
    if (inputs.positionDeg >= ForearmConstants.MAX_ANGLE_DEG
        && targetAngleDeg > inputs.positionDeg) {
      io.stop();
    }
  }

  /** Closed-loop position control */
  public void setAngle(double degrees) {
    targetAngleDeg =
        Math.max(ForearmConstants.MIN_ANGLE_DEG, Math.min(ForearmConstants.MAX_ANGLE_DEG, degrees));
    io.setPosition(targetAngleDeg);
  }

  /** Manual percent output control */
  public void runManual(double percent) {
    io.setPercent(percent);
  }

  /** Stop motor */
  public void stop() {
    io.stop();
  }

  /** Get current angle */
  public double getAngle() {
    return inputs.positionDeg;
  }
}
