package frc.robot.subsystems.climber;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.Timer;

public class ElevatorEncoderIOSim implements ElevatorEncoderIO {
  private edu.wpi.first.wpilibj.simulation.ElevatorSim sim;
  private double lastTime;
  private double lastDistance;
  LinearFilter elevatorFilter = LinearFilter.singlePoleIIR(0.1, 0.02);

  public ElevatorEncoderIOSim(edu.wpi.first.wpilibj.simulation.ElevatorSim sim) {
    this.sim = sim;
    this.lastTime = Timer.getFPGATimestamp();
    this.lastDistance = sim.getPositionMeters();
  }

  @Override
  public void updateInputs(ElevatorEncoderIO.ElevatorEncoderIOInputs inputs) {

    // 1:1 with the shaft
    inputs.unfilteredHeight = sim.getPositionMeters();
    inputs.height = elevatorFilter.calculate(inputs.unfilteredHeight);
    inputs.unfiliteredVelocity =
        (sim.getPositionMeters() - lastDistance) / (Timer.getFPGATimestamp() - lastTime);

    lastDistance = sim.getPositionMeters();
    lastTime = Timer.getFPGATimestamp();
  }

  @Override
  public void resetEncoderPositionWithLoadHeight() {
    sim.setState(0, 0);
  }
}
