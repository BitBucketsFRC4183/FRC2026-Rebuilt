package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.constants.ShooterConstants;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class ShooterSubsystem extends SubsystemBase {
  private final ShooterIO io;
  private boolean flywheelsRunning = false;

  private final ShooterIOInputsAutoLogged shooterInputs = new ShooterIOInputsAutoLogged();
  private final LoggedNetworkNumber targetVelocity =
      new LoggedNetworkNumber("Flywheel RPS", ShooterConstants.flywheelDefaultSpeed);
  private final SysIdRoutine sysId;
  private double storedDistance = -1;

  private final double[][] lookupTable =
      new double[][] {
        {0.0, 0.0},
        {5.0, 9.0},
        {6.0, 45.0}
      };

  public ShooterSubsystem(ShooterIO io) {
    this.io = io;
    // Configure SysId
    sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Flywheel/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> runCharacterization(voltage.in(Volts)), null, this));
  }

  public void runCharacterization(double voltage) {
    io.setFlywheelVoltage(voltage);
  }

  /** Returns a command to run a quasistatic test in the specified direction. */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return run(() -> runCharacterization(0.0))
        .withTimeout(1.0)
        .andThen(sysId.quasistatic(direction));
  }

  /** Returns a command to run a dynamic test in the specified direction. */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return run(() -> runCharacterization(0.0)).withTimeout(1.0).andThen(sysId.dynamic(direction));
  }

  public void calculateVelocity() {
    int index = 0;
    while (index < lookupTable.length) {
      if (lookupTable[index][0] >= storedDistance) {
        break;
      }
      index++;
    }
    if (lookupTable[index][0] != storedDistance) {
      // Calculates linear graph between 2 closest distances to estimate the best RPS to output
      System.out.println(lookupTable[index][1] - lookupTable[index - 1][1]);
      System.out.println(lookupTable[index][0] - lookupTable[index - 1][0]);
      double slope =
          (lookupTable[index][1] - lookupTable[index - 1][1])
              / (lookupTable[index][0] - lookupTable[index - 1][0]);
      //      targetVelocity =
      //          slope * (storedDistance - lookupTable[index - 1][0]) + lookupTable[index - 1][1];
      //    } else {
      //      targetVelocity = lookupTable[index][1];
    }
  }

  public void setTargetVelocity() {
    io.setFlywheelSpeed(targetVelocity.get());
    flywheelsRunning = true;
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
    flywheelsRunning = false;
    io.stopMotor();
  }

  public void startFeeding() {
    io.startFeeding();
  }

  // When Triggered Pressed, wait until true, then use motor to fire all the balls in storage
  // Operator is going to have one button, and they don't even have to hold it down :sob:
  public boolean targetReached() {
    return shooterInputs.flywheelVelocity < (targetVelocity.get() + ShooterConstants.tolerance)
        && shooterInputs.flywheelVelocity < (targetVelocity.get() - ShooterConstants.tolerance)
        && shooterInputs.flywheelVelocity2 < (targetVelocity.get() + ShooterConstants.tolerance)
        && shooterInputs.flywheelVelocity2 < (targetVelocity.get() - ShooterConstants.tolerance);
  }

  public boolean isFlywheelRunning() {
    return flywheelsRunning;
  }

  @Override
  public void periodic() {
    shooterInputs.targetFlywheelSpeed = targetVelocity.get();
    io.updateInputs(shooterInputs);
    Logger.processInputs("Flywheel", shooterInputs);
  }
}
