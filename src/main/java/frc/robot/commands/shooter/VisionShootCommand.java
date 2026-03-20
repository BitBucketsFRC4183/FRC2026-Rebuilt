package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.hopper.HopperSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;

public class VisionShootCommand extends Command {
  private final ShooterSubsystem shooter;
  private final HopperSubsystem hopper;
  private final Drive drive;
  private final VisionSubsystem vision;
  private boolean charged = false;
  private boolean stop = false;

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
    System.out.println("Started");
    charged = true;
    stop = false;
    shooter.setStoredDistance(vision.getHubDistanceMeter(drive::getPose));
  }

  @Override
  public void execute() {
    if (charged) {
      Commands.runOnce(shooter::charge);
      charged = false;
    } else {
      Commands.runOnce(shooter::calculateVelocity);
    }
    if (shooter.targetReached()) {
      ShooterCommands.startFeeding(shooter, hopper);
      stop = true;
    }
  }

  @Override
  public boolean isFinished() {
    return stop;
  }

  @Override
  public void end(boolean interrupted) {
    System.out.println("Command Finished");
    shooter.resetStoredDistance();
    shooter.stopFlywheel();
    shooter.stopIntermediateMotor();
    hopper.stopConveyor();
  }
}
