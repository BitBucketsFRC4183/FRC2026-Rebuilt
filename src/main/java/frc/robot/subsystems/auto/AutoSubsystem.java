package frc.robot.subsystems.auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.commands.ClimberCommands;
import frc.robot.commands.ShooterCommands;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.hopper.HopperSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.commands.IntakeCommands;

public class AutoSubsystem extends SubsystemBase {

  private final Drive drive;
  private final ShooterSubsystem shooter;
  private final ClimberSubsystem climber;
  private final HopperSubsystem hopper;
  private final IntakeSubsystem intake;
  // private ClimberSubsystem climberSubsystem;

  public AutoSubsystem(
          Drive drive, ShooterSubsystem shooter, ClimberSubsystem climber, HopperSubsystem hopper, IntakeSubsystem intake) {
    this.drive = drive;
    this.shooter = shooter;
    this.climber = climber;
    this.hopper = hopper;
    this.intake = intake;
      //this.intake = intake;
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

    NamedCommands.registerCommand("ExtendKickerbar", ExtendKickerbar());
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
    return ClimberCommands.increaseClimberLengthLevelOne(climber)
        .beforeStarting(() -> System.out.println("Climbing!"));
  }

  public Command ExtendKickerbar(){
    return IntakeCommands.moveServoTo90(intake);
  }

  // loading autoroutines from choreo to pathplanner
  public Command choreoPath(String trajName, boolean resetPose) {
    try {
      // Loading the path from the deploy/choreo folder
      PathPlannerPath path = PathPlannerPath.fromChoreoTrajectory(trajName);

      Command followCommand = AutoBuilder.followPath(path);

      if (resetPose) {
        return new InstantCommand(() -> drive.setPose(path.getStartingHolonomicPose().get()))
            .andThen(followCommand);
      } else {
        return followCommand;
      }
    } catch (Exception e) {
      System.err.println("Failed to load Choreo trajectory: " + trajName);
      e.printStackTrace();
      return Commands.none();
    }
  }
  // loads choreo path as an autobuilder command
  public Command cPath(String trajName, boolean doesReset) {
    PathPlannerPath path;

    try {
      // Load the path from the deploy/pathplanner folder by name
      path = PathPlannerPath.fromChoreoTrajectory(trajName);
    } catch (Exception e) {
      e.printStackTrace();
      return Commands.none();
    }
    // Create a path-following command using AutoBuilder
    if (doesReset) {
      return new InstantCommand(() -> drive.setPose(path.getStartingHolonomicPose().get()))
          .andThen(AutoBuilder.followPath(path));
    } else {
      return AutoBuilder.followPath(path);
    }
  }

  // loads choreo path as an autobuilder command, does not resetpose
  public Command cPath(String trajName) {
    PathPlannerPath path;

    try {
      // Load the path from the deploy/pathplanner folder by name
      path = PathPlannerPath.fromChoreoTrajectory(trajName);
    } catch (Exception e) {
      e.printStackTrace();
      return Commands.none();
    }
    // Create a path-following command using AutoBuilder
    return AutoBuilder.followPath(path);
  }
  /*
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
  goDepotToMid
    public Command () {
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

    public Command goBottomToOutpost() {
      PathPlannerPath BottomOutpost;

      try {
        BottomOutpost = PathPlannerPath.fromChoreoTrajectory("BottomToOutpost");
      } catch (Exception e) {
        e.printStackTrace();
        return Commands.none();
      }

      return AutoBuilder.followPath(BottomOutpost);
    }

    public Command goOutpostToShootBPs() {
      PathPlannerPath BottomShootPs;

      try {
        BottomShootPs = PathPlannerPath.fromChoreoTrajectory("OutpostToShooterB");
      } catch (Exception e) {
        e.printStackTrace();
        return Commands.none();
      }

      return AutoBuilder.followPath(BottomShootPs);
    }
  */
  public Command goToptoShooterPs() {
    return choreoPath("TopStartToShootT", true);
  }

  public Command goMidToShooterPs() {
    return choreoPath("MidStartToShootM", true);
  }

  public Command goMidTower() {
    return choreoPath("MidToTower", true);
  }

  public Command goBottomShootertoDepot() {
    return choreoPath("ShootMtoDepot", true);
  }

  public Command goMidShootertoDepot() {
    return choreoPath("ShootBtoDepot", true);
  }

  public Command goTopShootertoDepot() {
    return choreoPath("ShooterTtoDepot", true);
  }

  public Command goDepotToMid() {
    return choreoPath("DepotToMid", true);
  }

  public Command goBottomTower() {
    return choreoPath("BottomToTower", true);
  }

  public Command goTopTower() {
    return choreoPath("TopToTower", true);
  }

  public Command goShooterBtoTower() {
    return choreoPath("ShootBtoTower", true);
  }

  public Command goShooterTtoTower() {
    return choreoPath("ShootTtoTower", true);
  }

  public Command goBottomToOutpost() {
    return choreoPath("BottomToOutpost", true);
  }

  public Command goOutpostToShootBPs() {
    return choreoPath("OutpostToShooterB", true);
  }

  public Command goMidToDepot(){
    return choreoPath("MidToDepot", true);
  }

  public Command goTopStartToDepot(){
    return choreoPath("TopStartToDepot", true);
  }

  public Command goDepotToShootT(){
    return choreoPath("DepotToShootT", true);
  }

  public Command goBottomStartToShootB(){
    return choreoPath("BottomStartToShootB", true);
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
        new InstantCommand(()->System.out.println("extending kickerbar")),
        ExtendKickerbar(),
        new InstantCommand(() -> System.out.println("Moving from bottom position to Bottom shooting position")),
        goBottomStartToShootB(),
        new InstantCommand(() -> System.out.println("Reached bottom shooting position")),
        new InstantCommand(() -> System.out.println("We will now begin shooting")),
        shoot(),
        stop(),
        new InstantCommand(() -> System.out.println("routine complete")));
  }

  public Command topStartToShootOnly() {

    return Commands.sequence(
        new InstantCommand(()->System.out.println("extending kickerbar")),
        ExtendKickerbar(),
        new InstantCommand(() -> System.out.println("Moving from top position to shooting position")),
        goToptoShooterPs(),
        new InstantCommand(() -> System.out.println("Reached top shooting position")),
        new InstantCommand(() -> System.out.println("We will now begin shooting")),
        shoot(),
        stop(),
        new InstantCommand(() -> System.out.println("routine complete")));
  }

  public Command midStartToShootOnly() {
    return Commands.sequence(
        new InstantCommand(()->System.out.println("extending kickerbar")),
        ExtendKickerbar(),
        new InstantCommand(() -> System.out.println("Moving from mid position to shooting position")),
        goMidToShooterPs(),
        new InstantCommand(() -> System.out.println("Reached mid shooting position")),
        new InstantCommand(() -> System.out.println("We will now begin shooting")),
        shoot(),
        stop(),
        new InstantCommand(() -> System.out.println("routine complete")));
  }

  public Command StartBottomToTower() {
    return Commands.sequence(
        new InstantCommand(()->System.out.println("extending kickerbar")),
        ExtendKickerbar(),
        new InstantCommand(() -> System.out.println("We will just be climbing")),
        cPath("BottomToTower", true),
        climb(),
        stop(),
        new InstantCommand(() -> System.out.println("bottom to climb routine complete")));
  }

  public Command StartTopToTower() {
    return Commands.sequence(
        new InstantCommand(()->System.out.println("extending kickerbar")),
        ExtendKickerbar(),
        new InstantCommand(() -> System.out.println("We will just be climbing")),
        goTopTower(),
        climb(),
        stop(),
        new InstantCommand(() -> System.out.println("top to climb routine complete")));
  }

  public Command StartMidToTower() {
    return Commands.sequence(
        new InstantCommand(()->System.out.println("extending kickerbar")),
        ExtendKickerbar(),
        new InstantCommand(() -> System.out.println("We will just be climbing")),
        goMidTower(),
        climb(),
        stop(),
        new InstantCommand(() -> System.out.println("mid to climb routine complete")));
  }

  public Command StartTopShootEndL1() {
    return Commands.sequence(
        new InstantCommand(()->System.out.println("extending kickerbar")),
        ExtendKickerbar(),
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
        new InstantCommand(()->System.out.println("extending kickerbar")),
        ExtendKickerbar(),
        new InstantCommand(() -> System.out.println("We will move to bottom shooting position")),
        cPath("BottomStartToShootB", true),
        new InstantCommand(() -> System.out.println("Begin shooting")),
        shoot().withTimeout(6),
        new InstantCommand(() -> System.out.println("moving to tower to climb")),
        goShooterBtoTower(),
        climb());
  }

  public Command StartMidShootEndL1() {
    return Commands.sequence(
        new InstantCommand(()->System.out.println("extending kickerbar")),
        ExtendKickerbar(),
        new InstantCommand(() -> System.out.println("We will move to mid shooting position")),
        goMidToShooterPs(),
        new InstantCommand(() -> System.out.println("Begin shooting")),
        shoot().withTimeout(6),
        new InstantCommand(() -> System.out.println("moving to tower to climb")),
        goMidTower(),
        climb());
  }

  public Command StartBottomShootIntakeEndL1() {
    return Commands.sequence(
        new InstantCommand(()->System.out.println("extending kickerbar")),
        ExtendKickerbar(),
        new InstantCommand(() -> System.out.println("Routine 1 starting - ")),
        cPath("BottomStartToShootB", true),
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
        new InstantCommand(()->System.out.println("extending kickerbar")),
        ExtendKickerbar(),
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
        new InstantCommand(()->System.out.println("extending kickerbar")),
        ExtendKickerbar(),
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

  public Command StartBottomToOutpostShoot() {
    return Commands.sequence(
        new InstantCommand(()->System.out.println("extending kickerbar")),
        ExtendKickerbar(),
        new InstantCommand(() -> System.out.println("Moving from bottom start ps to outpost")),
        goBottomToOutpost(),
        new WaitCommand(6),
        new InstantCommand(() -> System.out.println("Moving to Shooter B Position")),
        goOutpostToShootBPs(),
        new InstantCommand(() -> System.out.println("We are going to shoot now")),
        shoot().withTimeout(6),
        stop(),
        new InstantCommand(() -> System.out.println("complete routine")));
  }
  public Command StartMidToDepotShoot(){
    return Commands.sequence(
            new InstantCommand(()->System.out.println("extending kickerbar")),
            ExtendKickerbar(),
            new InstantCommand(() -> System.out.println("Moving from mid starting ps to depot")),
            goMidToDepot(),
            new WaitCommand(6),
            new InstantCommand(()-> System.out.println("We are moving to shooting position")),
            goDepotToMid(),
            new InstantCommand(()->System.out.println("We are shooting now")),
            shoot().withTimeout(6),
            stop(),
            new InstantCommand(()->System.out.println("routine complete")));
  }

  public Command StartTopToDepotShoot(){
    return Commands.sequence(
            new InstantCommand(()->System.out.println("extending kickerbar")),
            ExtendKickerbar(),
            new InstantCommand(() -> System.out.println("Moving from top starting ps to depot")),
            goTopStartToDepot(),
            new WaitCommand(6),
            new InstantCommand(()-> System.out.println("We are moving to shooting position")),
            goDepotToShootT(),
            new InstantCommand(()->System.out.println("We are shooting now")),
            shoot().withTimeout(6),
            stop(),
            new InstantCommand(()->System.out.println("routine complete")));
  }
}
