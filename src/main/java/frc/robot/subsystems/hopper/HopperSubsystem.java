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
    Logger.processInputs("Hopper", inputs);
    io.updateInputs(inputs);
  }

  // Conveyor
  public void runConveyorForward() {
    io.setConveyorPercent(HopperConstants.CONVEYOR_FORWARD_PERCENT);
  }

  public void runConveyorReverse() {
    io.setConveyorPercent(HopperConstants.CONVEYOR_REVERSE_PERCENT);
  }

  public void stopConveyor() {
    io.stopConveyor();
  }

  // Telemetry
  public double getConveyorOutput() {
    return inputs.conveyorAppliedOutput;
  }
}
