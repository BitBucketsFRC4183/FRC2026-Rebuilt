package frc.robot.subsystems.auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.commands.ClimberCommands;
import frc.robot.commands.IntakeCommands;
import frc.robot.commands.ShooterCommands;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.hopper.HopperSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class AutoSubsystem extends SubsystemBase {

  private final Drive drive;
  private final ShooterSubsystem shooter;
  private final ClimberSubsystem climber;
  private final HopperSubsystem hopper;
  private final IntakeSubsystem intake;

  public AutoSubsystem(
      Drive drive,
      ShooterSubsystem shooter,
      ClimberSubsystem climber,
      HopperSubsystem hopper,
      IntakeSubsystem intake) {
    this.drive = drive;
    this.shooter = shooter;
    this.climber = climber;
    this.hopper = hopper;
    this.intake = intake;
    registerNamedCommands();
  }

  // Setup PathPlanner
  // Named Commands

  private void registerNamedCommands() {

    NamedCommands.registerCommand("Stop", stop());

    NamedCommands.registerCommand("Shoot", shoot());

    NamedCommands.registerCommand("Climb", climb());

    NamedCommands.registerCommand("ExtendKickerbar", extendKickerbar());
  }

  // Auto Actions
  public Command stop() {
    return Commands.runOnce(drive::stop, drive);
  }

  public Command shoot() {
    System.out.println("Beginning to Shoot");
    return ShooterCommands.shootAtRPS(10.0, shooter, hopper);
  }

  public Command climb() {
    return ClimberCommands.increaseClimberLengthLevelOne(climber)
        .beforeStarting(() -> System.out.println("Climbing!"));
  }

  public Command extendKickerbar() {
    System.out.println("extending kickerbar");
    return IntakeCommands.moveServoTo90(intake);
  }
  // Helper to drive while intaking
  private Command driveAndIntake(Command pathCommand) {
    return Commands.deadline(pathCommand, IntakeCommands.intake(intake));
  }

  private Command shootWithTimeout(double seconds) {
    return shoot().withTimeout(seconds);
  }

  /*public Command deployIntake() {
    return IntakeCommands.deploy(intake);
  }*/
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

  public Command goMidToDepot() {
    return choreoPath("MidToDepot", true);
  }

  public Command goTopStartToDepot() {
    return choreoPath("TopStartToDepot", true);
  }

  public Command goDepotToShootT() {
    return choreoPath("DepotToShootT", true);
  }

  public Command goBottomStartToShootB() {
    return choreoPath("BottomStartToShootB", true);
  }

  public Command goTopShootertoDepot() {
    return choreoPath("ShootTtoDepot", true);
  }
  // AUTOROUTINES
  public Command bottomStartToShootOnly() {
    return Commands.sequence(
        IntakeCommands.deploy(intake),
        extendKickerbar(),
        new InstantCommand(
            () -> System.out.println("Moving from bottom position to Bottom shooting position")),
        driveAndIntake(goBottomStartToShootB()),
        new InstantCommand(() -> System.out.println("Reached bottom shooting position")),
        shootWithTimeout(7),
        stop(),
        new InstantCommand(() -> System.out.println("routine complete")));
  }

  public Command topStartToShootOnly() {

    return Commands.sequence(
        IntakeCommands.deploy(intake),
        extendKickerbar(),
        new InstantCommand(
            () -> System.out.println("Moving from top position to shooting position")),
        driveAndIntake(goToptoShooterPs()),
        new InstantCommand(() -> System.out.println("Reached top shooting position")),
        shootWithTimeout(7),
        stop(),
        new InstantCommand(() -> System.out.println("routine complete")));
  }

  public Command midStartToShootOnly() {
    return Commands.sequence(
        IntakeCommands.deploy(intake),
        extendKickerbar(),
        new InstantCommand(
            () -> System.out.println("Moving from mid position to shooting position")),
        driveAndIntake(goMidToShooterPs()),
        new InstantCommand(() -> System.out.println("Reached mid shooting position")),
        shootWithTimeout(7),
        stop(),
        new InstantCommand(() -> System.out.println("routine complete")));
  }
  public Command StartBottomToOutpostShoot() {
    return Commands.sequence(
        IntakeCommands.deploy(intake),
        extendKickerbar(),
        new InstantCommand(() -> System.out.println("Moving from bottom start ps to outpost")),
        driveAndIntake(goBottomToOutpost()),
        new WaitCommand(6),
        new InstantCommand(() -> System.out.println("Moving to Shooter B Position")),
        driveAndIntake(goOutpostToShootBPs()),
        shootWithTimeout(6),
        stop(),
        new InstantCommand(() -> System.out.println("complete routine")));
  }

  public Command StartMidToDepotShoot() {
    return Commands.sequence(
        IntakeCommands.deploy(intake),
        extendKickerbar(),
        new InstantCommand(() -> System.out.println("Moving from mid starting ps to depot")),
        driveAndIntake(goMidToDepot()),
        new WaitCommand(6),
        new InstantCommand(() -> System.out.println("We are moving to shooting position")),
        driveAndIntake(goDepotToMid()),
        shootWithTimeout(6),
        stop(),
        new InstantCommand(() -> System.out.println("routine complete")));
  }

  public Command StartTopToDepotShoot() {
    return Commands.sequence(
        IntakeCommands.deploy(intake),
        extendKickerbar(),
        new InstantCommand(() -> System.out.println("Moving from top starting ps to depot")),
        driveAndIntake(goTopStartToDepot()),
        new WaitCommand(6),
        new InstantCommand(() -> System.out.println("We are moving to shooting position")),
        driveAndIntake(goDepotToShootT()),
        shootWithTimeout(6),
        stop(),
        new InstantCommand(() -> System.out.println("routine complete")));
  }

  public Command StartBottomShootOutpost() {
    return Commands.sequence(
        IntakeCommands.deploy(intake),
        extendKickerbar(),
        new InstantCommand(() -> System.out.println("Moving from bottom start to shoot ps")),
        driveAndIntake(goBottomStartToShootB()),
        shootWithTimeout(6),
        new InstantCommand(() -> System.out.println("We are moving to the outpost now")),
        driveAndIntake(goBottomShootertoDepot()),
        stop(),
        new InstantCommand(() -> System.out.println("routine complete")));
  }

  public Command StartMidShootDepot() {
    return Commands.sequence(
        IntakeCommands.deploy(intake),
        extendKickerbar(),
        new InstantCommand(() -> System.out.println("Moving from mid start to shooting ps")),
        driveAndIntake(goMidToShooterPs()),
        shootWithTimeout(6),
        new InstantCommand(() -> System.out.println("We are moving to the depot now")),
        driveAndIntake(goMidShootertoDepot()),
        stop(),
        new InstantCommand(() -> System.out.println("routine complete")));
  }

  public Command StartTopShootDepot() {
    return Commands.sequence(
        IntakeCommands.deploy(intake),
        extendKickerbar(),
        new InstantCommand(() -> System.out.println("Moving from top start to shooting ps")),
        driveAndIntake(goToptoShooterPs()),
        shootWithTimeout(6),
        new InstantCommand(() -> System.out.println("we are moving to depot now")),
        driveAndIntake(goTopShootertoDepot()),
        stop(),
        new InstantCommand(() -> System.out.println("routine complete")));
  }
}
