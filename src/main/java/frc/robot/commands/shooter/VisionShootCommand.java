package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.hopper.HopperSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;

public class VisionShootCommand extends Command {
  private ShooterSubsystem shooter;
  private final HopperSubsystem hopper;
  private final Drive drive;
  private final VisionSubsystem vision;
  private boolean charged = false;
  private boolean finished = false;

  public VisionShootCommand(
      ShooterSubsystem shooter, HopperSubsystem hopper, Drive drive, VisionSubsystem vision) {
    this.shooter = shooter;
    this.hopper = hopper;
    this.drive = drive;
    this.vision = vision;
    addRequirements(hopper);
    addRequirements(drive);
    addRequirements(vision);
    addRequirements(shooter);
  }

  @Override
  public void initialize() {
    charged = true;
    finished = false;
    shooter.setStoredDistance(vision.getHubDistanceMeter(drive::getPose));
  }

  @Override
  public void execute() {
    if (charged) {
      shooter.charge();
      charged = false;
    } else {
      shooter.calculateVelocity();
    }
    if (shooter.targetReached()) {
      ShooterCommands.startFeeding(shooter, hopper);
    }
  }

  public void stop() {
    finished = true;
  }

  public boolean isFinished() {
    return finished;
  }

  @Override
  public void end(boolean interrupted) {
    shooter.resetStoredDistance();
    shooter.stopFlywheel();
    shooter.stopIntermediateMotor();
    hopper.stopConveyor();
  }
}
