package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ShooterConstants;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class ShooterSubsystem extends SubsystemBase {
  private final ShooterIO io;

  private final ShooterIOInputsAutoLogged shooterInputs = new ShooterIOInputsAutoLogged();
  private final LoggedNetworkNumber targetVelocity = new LoggedNetworkNumber("Flywheel RPS", 9.0);

  private double storedDistance = -1;

  public ShooterSubsystem(ShooterIO io) {
    this.io = io;
  }

  public void calculateVelocity() {
    // Use the Lookup table once we have values
  }

  public void setTargetVelocity() {
    io.setSpeed(targetVelocity.get());
  }

  // Stores a distance to be used calculateTargetVelocity()
  public void setStoredDistance(double distance) {
    storedDistance = distance;
  }

  public boolean distanceStored() {
    return storedDistance > -1;
  }

  public void resetStoredDistance() {
    storedDistance = -1;
  }

  // Stops both Intermediate and Flywheel Motors
  public void stop() {
    io.stopMotor();
  }

  public void startFeeding() {
    io.startFeeding();
  }

  // insert code for setting hood angle stuff
  private void setHoodAngle(double angle) {
    angle = Math.max(ShooterConstants.minHoodAngle, Math.min(ShooterConstants.maxHoodAngle, angle));
    // finish it after figuring out how to
  }

  // When Triggered Pressed, wait until true, then use motor to fire all the balls in storage
  // Operator is going to have one button, and they don't even have to hold it down :sob:
  public boolean targetReached() {
    return shooterInputs.flywheelVelocity < (targetVelocity.get() + ShooterConstants.tolerance)
        && shooterInputs.flywheelVelocity < (targetVelocity.get() - ShooterConstants.tolerance)
        && shooterInputs.flywheelVelocity2 < (targetVelocity.get() + ShooterConstants.tolerance)
        && shooterInputs.flywheelVelocity2 < (targetVelocity.get() - ShooterConstants.tolerance);
  }

  @Override
  public void periodic() {
    io.updateInputs(shooterInputs);
    Logger.processInputs("Flywheel", shooterInputs);
  }
}
