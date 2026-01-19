package frc.robot.subsystems.hopper;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.HopperConstants;

public class HopperSubsystem extends SubsystemBase {

  private final HopperIO io;
  private final HopperIOInputs inputs = new HopperIOInputs();

  public HopperSubsystem(HopperIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
  }

  // Conveyor
  public void runConveyorForward() {
    io.setConveyorPercent(HopperConstants.CONVEYOR_FORWARD_PERCENT);
  }

  // Run in Reverse
  public void runConveyorReverse() {
    io.setConveyorPercent(HopperConstants.CONVEYOR_REVERSE_PERCENT);
  }

  public void stopConveyor() {
    io.stopConveyor();
  }

  // Spin Outtakes Forward
  public void runOuttakesForward() {
    io.setOuttakeLeftPercent(HopperConstants.OUTTAKE_FORWARD_PERCENT);
    io.setOuttakeRightPercent(HopperConstants.OUTTAKE_FORWARD_PERCENT);
  }

  // Spin Outtakes Reverse
  public void runOuttakesReverse() {
    io.setOuttakeLeftPercent(HopperConstants.OUTTAKE_REVERSE_PERCENT);
    io.setOuttakeRightPercent(HopperConstants.OUTTAKE_REVERSE_PERCENT);
  }

  public void runOuttakeLeft(double percent) {
    io.setOuttakeLeftPercent(percent);
  }

  public void runOuttakeRight(double percent) {
    io.setOuttakeRightPercent(percent);
  }

  // Stopboth
  public void stopOuttakes() {
    io.stopOuttakes();
  }

  // Telemetry

  public double getConveyorOutput() {
    return inputs.conveyorAppliedOutput;
  }

  public double getOuttakeLeftOutput() {
    return inputs.outtakeLeftAppliedOutput;
  }

  public double getOuttakeRightOutput() {
    return inputs.outtakeRightAppliedOutput;
  }
}
