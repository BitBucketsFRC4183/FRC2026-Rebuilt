package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
  ShooterIOTalonFX io;
  ShooterIOSparkMax ioSpark;

  private static double targetVelocity = 0;

  private double storedDistance = -1;

  public ShooterSubsystem(ShooterIOTalonFX io, ShooterIOSparkMax ioSpark) {
    this.io = io;
    this.ioSpark = ioSpark;
  }

  public static double getTargetVelocity() {
    return targetVelocity;
  }

  // Changes only the Flywheel Velocity, using storedDistance
  public void calculateVelocity() {
    double bestRPS = 0;
    boolean solutionFound = false;

    // Iterate through possible motor speeds (RPS)
    for (double testRPS = 0; testRPS <= ShooterConstants.maxRPS; testRPS += 0.5) {

      // Conversion to Rad/Sec
      double omega = testRPS * 2 * Math.PI;
      double vExit = omega * ShooterConstants.radius;

      double vX = vExit * Math.cos(Math.toRadians(ShooterConstants.shooterAngle));
      double vY = vExit * Math.sin(Math.toRadians(ShooterConstants.shooterAngle));

      double dy = ShooterConstants.hubHeight - ShooterConstants.shooterHeight;
      double g = -9.81;
      double discriminant = Math.pow(vY, 2) + 2 * g * dy;

      if (discriminant >= 0) {
        double t = (-vY - Math.sqrt(discriminant)) / g;
        double distanceX = vX * t;

        if (distanceX >= storedDistance) {
          bestRPS = testRPS;
          solutionFound = true;
          break; // Found the minimum speed needed to reach the distance
        }
      }
    }

    if (!solutionFound) {
      System.out.println("Range Error: Target out of reach!");
    } else {
      targetVelocity = bestRPS;
    }

    if(storedDistance == 0) {targetVelocity = 50;}
  }

  public void setTargetVelocity() {
    io.setSpeed(targetVelocity);
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
    ioSpark.stopMotor();
  }

  public void startIntermediateMotors() {
    // io.startIntermediateMotors();
    ioSpark.startIntermediateMotors();
  }

  // insert code for setting hood angle stuff
  private void setHoodAngle(double angle) {
    angle = Math.max(ShooterConstants.minHoodAngle, Math.min(ShooterConstants.maxHoodAngle, angle));
    // finish it after figuring out how to
  }

  // When Triggered Pressed, wait until true, then use motor to fire all the balls in storage
  // Operator is going to have one button, and they don't even have to hold it down :sob:
  public boolean targetReached() {
    return io.speedReached(targetVelocity);
  }
}
