package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
  ShooterIOTalonFX io;

  private static double targetVelocity = 0;

  private double storedDistance = 0;

  public ShooterSubsystem(ShooterIOTalonFX io) {
    this.io = io;
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

      // 1. Convert RPS to Tangential Velocity (v = r * omega)
      double omega = testRPS * 2 * Math.PI;
      double vExit = omega * ShooterConstants.radius;

      // 2. Split into X and Y components based on shooter mounting angle
      double vX = vExit * Math.cos(Math.toRadians(ShooterConstants.shooterAngle));
      double vY = vExit * Math.sin(Math.toRadians(ShooterConstants.shooterAngle));

      // 3. Projectile Motion: solve for time at the target height (y)
      // dy = vY*t + 0.5*g*t^2 -> 0.5gt^2 + vYt - dy = 0
      double dy = ShooterConstants.hubHeight - ShooterConstants.shooterHeight;
      double g = -9.81; // Use -32.2 if using feet

      double discriminant = Math.pow(vY, 2) + 2 * g * dy;

      if (discriminant >= 0) {
        // We only care about the further time point (the ball falling into the hub)
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
  }

  public void setTargetVelocity() {
    io.setSpeed(targetVelocity);
  }

  // Stores a distance to be used calculateTargetVelocity()
  public void setStoredDistance(double distance) {
    storedDistance = distance;
  }

  public boolean distanceStored() {
    return storedDistance > 0;
  }

  public void resetStoredDistance() {
    storedDistance = 0;
  }

  // Stops both Intermediate and Flywheel Motors
  public void stop() {
    io.stopMotor();
  }

  public void startIntermediateMotors() {
    io.startIntermediateMotors();
  }

  public boolean finishedCalculations() {
    return targetVelocity != 0;
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
