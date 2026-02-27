package frc.robot.subsystems.hopper;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;

public class HopperIOSim implements HopperIO {

  private double motorVelocityRPS = 0.0;
  private double motorCurrent = 0.0;
  private double targetVelocity = 0.0;

  private static final double MAX_RPS = 100.0;
  private static final double MAX_CURRENT = 40.0;
  private static final double RESPONSE = 10.0;

  private double lastTime = Timer.getFPGATimestamp();

  @Override
  public void updateInputs(HopperIOInputs inputs) {
    double now = Timer.getFPGATimestamp();
    double dt = now - lastTime;
    lastTime = now;

    double target = targetVelocity;

    motorVelocityRPS += (target - motorVelocityRPS) * MathUtil.clamp(dt * RESPONSE, 0.0, 1.0);

    motorCurrent = Math.abs(motorVelocityRPS / MAX_RPS) * MAX_CURRENT;

    inputs.motorVelocityRPS = motorVelocityRPS;
    inputs.motorCurrentAmps = motorCurrent;
    inputs.motorTargetVelocityRPS = targetVelocity;
  }

  @Override
  public void setVelocity(double rps) {
    targetVelocity = MathUtil.clamp(rps, -MAX_RPS, MAX_RPS);
  }
}
