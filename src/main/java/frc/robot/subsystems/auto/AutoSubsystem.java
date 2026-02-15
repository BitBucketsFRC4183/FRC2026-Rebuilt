package frc.robot.subsystems.auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.ClimberCommands;
import frc.robot.commands.ShooterCommands;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.hopper.HopperSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import org.json.simple.parser.ParseException;

import java.io.IOException;

public class AutoSubsystem extends SubsystemBase {

  private final DriveSubsystem drive;
  private final ShooterSubsystem shooter;
  private final ClimberSubsystem climber;
  private final HopperSubsystem hopper;
  // private ClimberSubsystem climberSubsystem;

  public AutoSubsystem(
      DriveSubsystem drive,
      ShooterSubsystem shooter,
      ClimberSubsystem climber,
      HopperSubsystem hopper) {
    this.drive = drive;
    this.shooter = shooter;
    this.climber = climber;
    this.hopper = hopper;
    // this.autoFactory = new AutoFactory(drive::getPose, drive::setPose,
    // drive::followTrajectorySample, false, drive, trajectoryLogger())
    registerNamedCommands();
  }

  // PathPlannerPath MidStarttoMid = PathPlannerPath.fromChoreoTrajectory("MidStarttoMid");
  // Setup PathPlanner
  // Named Commands

  private void registerNamedCommands() {

    NamedCommands.registerCommand("Stop", stop());

    NamedCommands.registerCommand("Shoot", shoot());

    NamedCommands.registerCommand("Climb", climb());
  }

  /** Stops drivetrain */
  // public Command stop() {
  //  return drive.run(() -> drive.drive(new edu.wpi.first.math.kinematics.ChassisSpeeds()));
  // }

  // Auto Actions
  public Command stop() {
    return Commands.runOnce(drive::stop, drive);
  }

  public Command shoot() {
    return ShooterCommands.revFlywheels(shooter, hopper);
  }

  public Command climb() {
    return ClimberCommands.increaseClimberLength(climber)
        .beforeStarting(() -> System.out.println("Climbing!"));
  }

  // autoroutines

  /**
   * TODO: NAVYA BELOW IS THE CORRECT WAY TO MAKE A PATHPLANNER COMMAND. YOU NEED TO CHANGE ALL OF
   * YOUR COMMANDS TO MATCH "goBottomToMid"'S FORMATTING AND THEN YOU MUST TEST YOUR PATHS IN SIM!!
   * PLEASSSEE!!!
   */
  public Command goBottomToShootPs() {
    PathPlannerPath path1;

    try {
      // Load the path from the deploy/pathplanner folder by name
      path1 = PathPlannerPath.fromChoreoTrajectory("BottomStartToShootB");
    } catch (Exception e) {
      e.printStackTrace();
      return Commands.none();
    }

    // Create a path-following command using AutoBuilder
    return AutoBuilder.followPath(path1);
  }
  public Command goToptoShooterPs() {
    PathPlannerPath path2;

    try {
      // Load the path from the deploy/pathplanner folder by name
      path2 = PathPlannerPath.fromChoreoTrajectory("TopStartToShootT");
    } catch (Exception e) {
      e.printStackTrace();
      return Commands.none();
    }

    // Create a path-following command using AutoBuilder
    return AutoBuilder.followPath(path2);
  }

  public Command goMidToShooterPs() {
    PathPlannerPath path3;

    try {
      path3 = PathPlannerPath.fromChoreoTrajectory("MidStartToShootM");
    } catch (Exception e) {
      e.printStackTrace();
      return Commands.none();
    }

    return AutoBuilder.followPath(path3);
  }

  // climbing only
  public Command goMidtoTower() {
    PathPlannerPath path4;

    try {
      path4 = PathPlannerPath.fromChoreoTrajectory("MidtoTower");
    } catch (Exception e) {
      e.printStackTrace();
      return Commands.none();
    }

    return AutoBuilder.followPath(path4);
  }

  public Command goBottomShootertoDepot() {
    PathPlannerPath path5;

    try {
      path5 = PathPlannerPath.fromChoreoTrajectory("ShootMtoDepot");
    } catch (Exception e) {
      e.printStackTrace();
      return Commands.none();
    }

    return AutoBuilder.followPath(path5);
  }

  public Command goMidShootertoDepot() {
    PathPlannerPath path6;

    try {
      path6 = PathPlannerPath.fromChoreoTrajectory("ShootBtoDepot");
    } catch (Exception e) {
      e.printStackTrace();
      return Commands.none();
    }

    return AutoBuilder.followPath(path6);
  }

  public Command goTopShootertoDepot() {
    PathPlannerPath path7;

    try {
      path7 = PathPlannerPath.fromChoreoTrajectory("ShootTtoDepot");
    } catch (Exception e) {
      e.printStackTrace();
      return Commands.none();
    }

    return AutoBuilder.followPath(path7);
  }

  public Command goDepotToMid() {
    PathPlannerPath path8;

    try {
      path8 = PathPlannerPath.fromChoreoTrajectory("DepotToMid");
    } catch (Exception e) {
      e.printStackTrace();
      return Commands.none();
    }

    return AutoBuilder.followPath(path8);
  }

  public Command goMidTower() {
    PathPlannerPath MdTower;

    try {
      MdTower = PathPlannerPath.fromChoreoTrajectory("MidtoTower");
    } catch (Exception e) {
      e.printStackTrace();
      return Commands.none();
    }

    return AutoBuilder.followPath(MdTower);
  }

  public Command goBottomTower() {
    PathPlannerPath BtTower;

    try {
      BtTower = PathPlannerPath.fromChoreoTrajectory("BottomToTower");
    } catch (Exception e) {
      e.printStackTrace();
      return Commands.none();
    }

    return AutoBuilder.followPath(BtTower);
  }

  public Command goTopTower() {
    PathPlannerPath TpTower;

    try {
      TpTower = PathPlannerPath.fromChoreoTrajectory("TopToTower");
    } catch (Exception e) {
      e.printStackTrace();
      return Commands.none();
    }

    return AutoBuilder.followPath(TpTower);
  }

  public Command goShooterBtoTower() {
    PathPlannerPath ShooterBTower;

    try {
      ShooterBTower = PathPlannerPath.fromChoreoTrajectory("ShootBtoTower");
    } catch (Exception e) {
      e.printStackTrace();
      return Commands.none();
    }

    return AutoBuilder.followPath(ShooterBTower);
  }

  public Command goShooterTtoTower() {
    PathPlannerPath ShooterTTower;

    try {
      ShooterTTower = PathPlannerPath.fromChoreoTrajectory("ShootTtoTower");
    } catch (Exception e) {
      e.printStackTrace();
      return Commands.none();
    }

    return AutoBuilder.followPath(ShooterTTower);
  }

  // AUTOROUTINES

  /*public Command bottomStartToShootOnly() {
    return Commands.sequence(
        // FIRST: Reset robot pose to the path's starting position
        new InstantCommand(
            () -> {
              try {
                // Load the path to get its starting pose
                PathPlannerPath path = PathPlannerPath.fromChoreoTrajectory("BottomStartToShootB");
                Pose2d startPose = path.getPathPoses().get(0);
                drive.setPose(startPose);
                System.out.println("Reset pose to: " + startPose);
              } catch (Exception e) {
                System.err.println("Failed to reset pose: " + e.getMessage());
              }
            }),
        new InstantCommand(
            () -> System.out.println("Moving from bottom position to Bottom shooting position")),
        goBottomToShootPs(),
        new InstantCommand(() -> System.out.println("Reached bottom shooting position")),
        shoot(),
        stop(),
        new InstantCommand(() -> System.out.println("routine complete")));
  } */
  public Command bottomStartToShootOnly() {
    return Commands.sequence(
        new InstantCommand(
            () -> System.out.println("Moving from bottom position to Bottom shooting position")),
        goBottomToShootPs(),
        new InstantCommand(() -> System.out.println("Reached bottom shooting position")),
        new InstantCommand(() -> System.out.println("We will now begin shooting")),
        shoot(),
        stop(),
        new InstantCommand(() -> System.out.println("routine complete")));
  }

  public Command topStartToShootOnly() {
    return Commands.sequence(
        new InstantCommand(
            () -> System.out.println("Moving from top position to shooting position")),
        goToptoShooterPs(),
        new InstantCommand(() -> System.out.println("Reached top shooting position")),
        new InstantCommand(() -> System.out.println("We will now begin shooting")),
        shoot(),
        stop(),
        new InstantCommand(() -> System.out.println("routine complete")));
  }

  public Command midStartToShootOnly() {
    return Commands.sequence(
        new InstantCommand(
            () -> System.out.println("Moving from mid position to shooting position")),
        goMidToShooterPs(),
        new InstantCommand(() -> System.out.println("Reached mid shooting position")),
        new InstantCommand(() -> System.out.println("We will now begin shooting")),
        shoot(),
        stop(),
        new InstantCommand(() -> System.out.println("routine complete")));
  }

  public Command StartBottomToTower() {
    return Commands.sequence(
        new InstantCommand(() -> System.out.println("We will just be climbing")),
        goBottomTower(),
        climb(),
        stop(),
        new InstantCommand(() -> System.out.println("bottom to climb routine complete")));
  }

  public Command StartTopToTower() {
    return Commands.sequence(
        new InstantCommand(() -> System.out.println("We will just be climbing")),
        goTopTower(),
        climb(),
        stop(),
        new InstantCommand(() -> System.out.println("top to climb routine complete")));
  }

  public Command StartMidToTower() {
    return Commands.sequence(
        new InstantCommand(() -> System.out.println("We will just be climbing")),
        goMidTower(),
        climb(),
        stop(),
        new InstantCommand(() -> System.out.println("mid to climb routine complete")));
  }

  public Command StartTopShootEndL1() {
    return Commands.sequence(
        new InstantCommand(() -> System.out.println("We will move to top shooting position")),
        goToptoShooterPs(),
        new InstantCommand(() -> System.out.println("Begin shooting")),
        shoot().withTimeout(6),
        new InstantCommand(() -> System.out.println("moving to tower to climb")),
        goShooterTtoTower(),
        climb());
  }

  public Command StartBottomShootEndL1() {
    return Commands.sequence(
        new InstantCommand(() -> System.out.println("We will move to bottom shooting position")),
        goBottomToShootPs(),
        new InstantCommand(() -> System.out.println("Begin shooting")),
        shoot().withTimeout(6),
        new InstantCommand(() -> System.out.println("moving to tower to climb")),
        goShooterBtoTower(),
        climb());
  }

  public Command StartMidShootEndL1() {
    return Commands.sequence(
        new InstantCommand(() -> System.out.println("We will move to mid shooting position")),
        goMidToShooterPs(),
        new InstantCommand(() -> System.out.println("Begin shooting")),
        shoot().withTimeout(6),
        new InstantCommand(() -> System.out.println("moving to tower to climb")),
        goMidtoTower(),
        climb());
  }

  public Command StartBottomShootIntakeEndL1() {
    return Commands.sequence(
        new InstantCommand(() -> System.out.println("Routine 1 starting - ")),
        goBottomToShootPs(),
        shoot().withTimeout(4),
        goBottomShootertoDepot(),
        new WaitCommand(3),
        goDepotToMid(),
        goMidTower(),
        climb(),
        stop(),
        new InstantCommand(() -> System.out.println("routine 1 complete")));
  }

  public Command StartMidShootIntakeEndL1() {
    return Commands.sequence(
        new InstantCommand(() -> System.out.println("Moving to Shooter position")),
        goMidToShooterPs(),
        shoot().withTimeout(4),
        goMidShootertoDepot(),
        new WaitCommand(3),
        goDepotToMid(),
        goMidTower(),
        climb(),
        stop(),
        new InstantCommand(() -> System.out.println("complete routine")));
  }

  public Command StartTopShootIntakeEndL1() {
    return Commands.sequence(
        new InstantCommand(() -> System.out.println("Moving from top to mid shooting position")),
        goToptoShooterPs(),
        shoot().withTimeout(4),
        goTopShootertoDepot(),
        new WaitCommand(3),
        goDepotToMid(),
        goMidTower(),
        climb(),
        stop(),
        new InstantCommand(() -> System.out.println("complete routine")));
  }
}
