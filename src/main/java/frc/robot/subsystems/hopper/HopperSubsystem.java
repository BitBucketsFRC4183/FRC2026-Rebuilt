package frc.robot.subsystems.hopper;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.HopperConstants;
import org.littletonrobotics.junction.Logger;

public class HopperSubsystem extends SubsystemBase {

  private final HopperIO io;
  private final HopperIOInputsAutoLogged inputs = new HopperIOInputsAutoLogged();

  public HopperSubsystem(HopperIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Hopper", inputs);
  }

  public void runConveyorForward() {
    io.setVelocity(HopperConstants.FORWARD_RPS);
  }

  public void runConveyorReverse() {
    io.setVelocity(HopperConstants.REVERSE_RPS);
  }

  public void stopConveyor() {
    io.stopMotor();
  }

  public double getVelocity() {
    return inputs.motorVelocityRPS;
  }
}
