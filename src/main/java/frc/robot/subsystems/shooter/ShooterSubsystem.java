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

  private final double[][] lookupTable =
      new double[][] {
        {0.0, 0.0},
        {5.0, 9.0},
        {10.0, 15.0}
      };

  public ShooterSubsystem(ShooterIO io) {
    this.io = io;
    System.out.println(lookupTable.length);
  }

  public void calculateVelocity() {
    int index = 0;
    while(index < lookupTable.length) {
      if (lookupTable[index][0] >= storedDistance) {
        break;
      }
      index++;
    }
    if(lookupTable[index][0] != storedDistance) {
      //Calculates linear graph between 2 closest distances to estimate the best RPS to output
      double slope = (lookupTable[index][1] - lookupTable[index - 1][1]) / (lookupTable[index][0] - lookupTable[index - 1][0]);
      targetVelocity.set(slope * (storedDistance - lookupTable[index - 1][0]) + lookupTable[index - 1][1]);
    }
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
