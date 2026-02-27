package frc.robot.subsystems.power_distribution;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class PowerDistributionSubsystem extends SubsystemBase {

  private final IntakeSubsystem intake;
  private final ShooterSubsystem shooter;

  private PowerState currentState = PowerState.FULL_DRIVE;
  private boolean overrideActive = false;
  public double driveFactor = 1.0;

  public PowerDistributionSubsystem(IntakeSubsystem intake, ShooterSubsystem shooter) {
    this.intake = intake;
    this.shooter = shooter;
  }

  @Override
  public void periodic() {
    boolean intakeRunning = intake.isRunning();
    boolean shooterRunning = shooter.isFlywheelRunning();

    if (!overrideActive) {
      if (intakeRunning && shooterRunning) {
        currentState = PowerState.INTAKE_AND_SHOOT_DRIVE;
      } else if (shooterRunning) {
        currentState = PowerState.SHOOTING_DRIVE;
      } else if (intakeRunning) {
        currentState = PowerState.INTAKE_DRIVE;
      } else {
        currentState = PowerState.FULL_DRIVE;
      }
    } else {
      currentState = PowerState.DRIVE_OVERRIDE;
    }

    switch (currentState) {
      case INTAKE_DRIVE:
        driveFactor = 0.7;
        break;

      case SHOOTING_DRIVE:
      case INTAKE_AND_SHOOT_DRIVE:
        driveFactor = 0.2;
        break;

      case DRIVE_OVERRIDE:
        driveFactor = 1.0;
        break;

      case FULL_DRIVE:
      default:
        driveFactor = 1.0;
        break;
    }

    SmartDashboard.putString("PowerState", currentState.name());
    SmartDashboard.putNumber("DriveFactor", driveFactor);
  }

  public void setOverride(boolean enabled) {
    overrideActive = enabled;
  }

  public double getDriveFactor() {
    return driveFactor;
  }

  public PowerState getState() {
    return currentState;
  }
}
