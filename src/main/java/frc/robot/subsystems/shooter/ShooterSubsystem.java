package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.constants.ShooterConstants;
import org.littletonrobotics.junction.Logger;

public class ShooterSubsystem extends SubsystemBase {
  private final ShooterIO io;
  private boolean flywheelsRunning = false;
  private boolean passing = false;

  private final ShooterIOInputsAutoLogged shooterInputs = new ShooterIOInputsAutoLogged();
  private static double targetVelocity = 0;
  private final SysIdRoutine sysId;
  private static double storedDistance = 0;
  private boolean dataReceived = false;
  // FIRST column is distances (in meters), second column is RPS
  private final double[][] lookupTable =
      new double[][] {
        {0, 0},
        {1.397 + ShooterConstants.distanceOffset, 41.0},
        {1.8034 + ShooterConstants.distanceOffset, 43.0},
        {2.0828 + ShooterConstants.distanceOffset, 44.0},
        {2.4638 + ShooterConstants.distanceOffset, 46.0},
        {2.7432 + ShooterConstants.distanceOffset, 47.0},
        {2.9718 + ShooterConstants.distanceOffset, 49.0},
        {3.2766 + ShooterConstants.distanceOffset, 51.0},
        {3.6068 + ShooterConstants.distanceOffset, 53.0},
        {4.1148 + ShooterConstants.distanceOffset, 54.0},
      };

  public ShooterSubsystem(ShooterIO io) {
    this.io = io;
    // Configure SysId
    sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                Volts.of(7),
                null,
                (state) -> SignalLogger.writeString("state", state.toString())),
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
    // Linear Regression
    targetVelocity =
        0.13
                * storedDistance
                // Conversion to Inches lol
                * 39.3701
            + 28.9;
    if (passing) {
      targetVelocity = (ShooterConstants.defaultPassingSpeed);
    }
    setTargetVelocity(targetVelocity);
  }

  public void setTargetVelocity(double targetVelocity) {
    io.setFlywheelSpeed(targetVelocity);
    flywheelsRunning = true;
    ShooterSubsystem.targetVelocity = (targetVelocity);
  }

  // Stores a distance to be used calculateTargetVelocity()
  public void setStoredDistance(double distance) {
    storedDistance = distance;
    dataReceived = true;
    calculateVelocity();
  }

  public void resetStoredDistance() {
    storedDistance = 0;
  }

  // Stops both Intermediate and Flywheel Motors
  public void stopFlywheel() {
    flywheelsRunning = false;
    io.stopFlywheel();
    dataReceived = false;
  }

  public void startIntermediateMotor() {
    io.startIntermediateMotor();
  }

  public void stopIntermediateMotor() {
    io.stopIntermediateMotor();
  }

  public void charge() {
    io.setFlywheelVoltage(12);
  }

  // When Triggered Pressed, wait until true, then use motor to fire all the balls in storage
  public boolean targetReached() {
    return shooterInputs.flywheelVelocity >= (targetVelocity - ShooterConstants.tolerance)
        && shooterInputs.flywheelVelocity2 >= (targetVelocity - ShooterConstants.tolerance);
  }

  public boolean isFlywheelRunning() {
    return flywheelsRunning;
  }

  public static double getTargetVelocity() {
    return targetVelocity;
  }

  @Override
  public void periodic() {
    Logger.recordOutput("Vision Data Received", dataReceived);
    io.updateInputs(shooterInputs);
    Logger.processInputs("Flywheel", shooterInputs);
    Logger.recordOutput("Target Velocity", targetVelocity);
  }

  public static double getStoredDistance() {
    return storedDistance;
  }

  public void switchPassingMode() {
    passing = !passing;
  }
}
