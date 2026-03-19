package frc.robot.commands.shooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.hopper.HopperSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;

import java.util.function.Supplier;

import static edu.wpi.first.wpilibj2.command.Commands.waitSeconds;
import static edu.wpi.first.wpilibj2.command.Commands.waitUntil;
import static frc.robot.commands.shooter.ShooterCommands.startFeeding;

public class VisionShootCommand extends Command {
    private final ShooterSubsystem shooter;
    private final HopperSubsystem hopper;
    private final Drive drive;
    private final VisionSubsystem vision;

    public VisionShootCommand(ShooterSubsystem shooter, HopperSubsystem hopper, Drive drive, VisionSubsystem vision) {
        this.shooter = shooter;
        this.hopper = hopper;
        this.drive = drive;
        this.vision = vision;
        addRequirements(hopper);
        addRequirements(drive);
        addRequirements(vision);
        addRequirements(shooter);
    }

    public void initialize() {
        shooter.setStoredDistance(vision.getHubDistanceMeter(drive::getPose));
        waitUntil(shooter::targetReached);
        waitSeconds(0.8);
        startFeeding(shooter, hopper);
        shooter.charge();
        waitSeconds(0.02);
        shooter.calculateVelocity();
        withTimeout(2.0);
    }

    public void end(boolean interrupted) {
        shooter.resetStoredDistance();
        shooter.stopFlywheel();
        shooter.stopIntermediateMotor();
        hopper.stopConveyor();
    }
}
