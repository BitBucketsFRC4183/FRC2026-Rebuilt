package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
  ShooterIOTalonFX io;

  private int targetVelocity = 3000;

  private double storedDistance = 0;

  public ShooterSubsystem(ShooterIOTalonFX io) {
    this.io = io;
  }

  public void setTargetFlywheelVelocity() {
    targetVelocity = 0;
    for (double distanceAchieved = 0; distanceAchieved >= storedDistance; targetVelocity += 2) {
      //Calculating tangential velocity
      double yVelocity =
              targetVelocity * ShooterConstants.radius * Math.sin(Math.toRadians(ShooterConstants.shooterAngle));
      // Using Kinematics to calculate RPM to launch the ball, hopefully air resistance is
      // negligible lol
      double h = ShooterConstants.hubHeight - ShooterConstants.shooterHeight;
      double airTime =
              (-yVelocity
              + (Math.sqrt(Math.pow(yVelocity, 2) - 2 * ShooterConstants.gravity * h)))
                  / ShooterConstants.gravity;
      distanceAchieved =
          airTime
              * targetVelocity
              * ShooterConstants.radius
              * Math.cos(Math.toRadians(ShooterConstants.shooterAngle));
      // If it reaches this statement, the bot is too far from the hub to make it in or the
      // shooterAngle is not appropriate
      if (targetVelocity > ShooterConstants.maxRPM * 2 * Math.PI / 60) {
        break;
      }
      io.setSpeed(targetVelocity);
    }
  }

  // The Distance inputted should be the distance of the bot from the "center" of the hub

  public void setTargetHoodAngle(double distance) {
    double targetFlywheelRPM = 3000;
    // ill change later i j chose a value for now
    double launchVelocity = (targetFlywheelRPM * 2 * Math.PI / 60) * ShooterConstants.radius;
    double verticalDistance = ShooterConstants.hubHeight - ShooterConstants.shooterHeight;
    // initialize velocity squared - physics variables
    // double v0squared = Math.pow(v0, 2);
    // 6double v0fourth = Math.pow(v0, 4);

    double targetHoodAngle = ShooterConstants.minHoodAngle;
    // change accuracy for min and max hood angle later
    for (double angle = ShooterConstants.minHoodAngle;
        angle <= ShooterConstants.maxHoodAngle;
        angle += 0.01) {
      double launchVelX = launchVelocity * Math.cos(angle);
      double launchVelY = launchVelocity * Math.sin(angle);

      double discriminant =
          Math.pow(launchVelY, 2) + 2 * ShooterConstants.gravity * verticalDistance;
      // discriminant = launch energy - energy needed for dist + height - theory
      if (discriminant >= 0) {
        double airtime = (launchVelY + Math.sqrt(discriminant)) / ShooterConstants.gravity;

        double distanceAchieved = launchVelX * airtime;

        if (Math.abs(distanceAchieved - distance) <= ShooterConstants.error) {
          targetHoodAngle = angle;
        }
      }
      //Commented this out for now, this method is for a given distance, not for a Flywheel Velocity
      //setTargetFlywheelVelocity(targetFlywheelRPM);

      //Instead use this:
      io.setSpeed(targetVelocity);
    }
  }

  public void setStoredDistance(double distance) {storedDistance = distance;}

  public boolean distanceStored() {
    return storedDistance > 0;
  }

  public void resetStoredDistance() {storedDistance = 0;}

  public void stop() {io.stopMotor();}

  public void startIntermediateMotors() {io.startIntermediateMotors();}

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
