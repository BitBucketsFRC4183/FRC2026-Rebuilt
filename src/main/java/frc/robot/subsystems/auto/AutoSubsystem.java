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
  public enum ShootingPosition {
    POSITION_mid(7.73955, 32.74425),
    POSITION_top(7.47391, 31.62039),
    POSITION_btm(9.66585, 40.89397);
    public final double distance;
    public final double velocity;

    ShootingPosition(double distance, double velocity) {
      this.distance = distance;
      this.velocity = velocity;
    }
  }

  private void registerNamedCommands() {

    NamedCommands.registerCommand("Stop", stop());

    NamedCommands.registerCommand("ShootMid", shoot(ShootingPosition.POSITION_mid));
    NamedCommands.registerCommand("ShootTop", shoot(ShootingPosition.POSITION_top));
    NamedCommands.registerCommand("ShootBtm", shoot(ShootingPosition.POSITION_btm));

    NamedCommands.registerCommand("Climb", climb());

    NamedCommands.registerCommand("ExtendKickerbar", extendKickerbar());
  }

  // Auto Actions
  public Command stop() {
    return Commands.runOnce(drive::stop, drive);
  }

  public Command shoot(ShootingPosition position) {
    System.out.println("Beginning to Shoot");
    return ShooterCommands.shootAtRPS(position.velocity, shooter, hopper);
  }
  // 40.89397 for bottom shooting ps
  // 31.62039 for top shooting ps
  // 32.74425 for mid shooting ps
  public Command climb() {
    return ClimberCommands.increaseClimberLengthLevelOne(climber)
        .beforeStarting(() -> System.out.println("Climbing!"));
  }

  public Command extendKickerbar() {
    System.out.println("extending kickerbar");
    return IntakeCommands.moveServoTo90(intake);
  }
  // Helper to drive while intaking
  /*private Command driveAndIntake(Command pathCommand) {
    return Commands.deadline(pathCommand, IntakeCommands.intake(intake));
  }*/
  // outtake in alliance zone
  private Command outtakeAtAlliance() {
    System.out.println("dumping balls in alliance zone!");
    return IntakeCommands.outtake(intake);
  }
  // momentary intake
  private Command IntakeOnly() {
    System.out.println("intaking at neutral zone");
    return IntakeCommands.intake(intake);
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

  //  public Command goMidTower() {
  //    return choreoPath("MidToTower", true);
  //  }

  public Command goBottomShootertoDepot() {
    return choreoPath("ShootBtoDepot", true);
  }

  public Command goMidShootertoDepot() {
    return choreoPath("ShootMtoDepot", true);
  }

  public Command goDepotToMid() {
    return choreoPath("DepotToMid", true);
  }

  //  public Command goBottomTower() {
  //    return choreoPath("BottomToTower", true);
  //  }

  //  public Command goTopTower() {
  //    return choreoPath("TopToTower", true);
  //  }

  //  public Command goShooterBtoTower() {
  //    return choreoPath("ShootBtoTower", true);
  //  }

  //  public Command goShooterTtoTower() {
  //    return choreoPath("ShootTtoTower", true);
  //  }

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

  public Command goBottomStartToNeutralZ() {
    return choreoPath("bottomStartToneutralZ", true);
  }

  public Command goIntakeBtmToAlliance() {
    return choreoPath("IntakeBtmToAlliance", true);
  }

  public Command goTopStartToneutralZ() {
    return choreoPath("topStartToneutralZ", true);
  }

  public Command goIntakeTopToAlliance() {
    return choreoPath("IntakeToptoAlliance", true);
  }
  // AUTOROUTINES
  public Command bottomStartToShootOnly() {
    return Commands.sequence(
        IntakeCommands.deploy(intake),
        extendKickerbar(),
        new InstantCommand(
            () -> System.out.println("Moving from bottom position to Bottom shooting position")),
        Commands.parallel(goBottomStartToShootB(), IntakeCommands.intake(intake)).withTimeout(2),
        new InstantCommand(() -> System.out.println("Reached bottom shooting position")),
        shoot(ShootingPosition.POSITION_btm).withTimeout(7),
        stop(),
        new InstantCommand(() -> System.out.println("routine complete")));
  }

  public Command topStartToShootOnly() {

    return Commands.sequence(
        IntakeCommands.deploy(intake),
        extendKickerbar(),
        new InstantCommand(
            () -> System.out.println("Moving from top position to shooting position")),
        Commands.parallel(goToptoShooterPs(), IntakeCommands.intake(intake)).withTimeout(2),
        new InstantCommand(() -> System.out.println("Reached top shooting position")),
        shoot(ShootingPosition.POSITION_top).withTimeout(7),
        stop(),
        new InstantCommand(() -> System.out.println("routine complete")));
  }

  public Command midStartToShootOnly() {
    return Commands.sequence(
        IntakeCommands.deploy(intake),
        extendKickerbar(),
        new InstantCommand(
            () -> System.out.println("Moving from mid position to shooting position")),
        Commands.parallel(goMidToShooterPs(), IntakeCommands.intake(intake)).withTimeout(2),
        new InstantCommand(() -> System.out.println("Reached mid shooting position")),
        shoot(ShootingPosition.POSITION_mid).withTimeout(7),
        stop(),
        new InstantCommand(() -> System.out.println("routine complete")));
  }

  public Command StartBottomToOutpostShoot() {
    return Commands.sequence(
        IntakeCommands.deploy(intake),
        extendKickerbar(),
        new InstantCommand(() -> System.out.println("Moving from bottom start ps to outpost")),
        Commands.parallel(goBottomToOutpost(), IntakeCommands.intake(intake)).withTimeout(2),
        new WaitCommand(6),
        new InstantCommand(() -> System.out.println("Moving to Shooter B Position")),
        Commands.parallel(goOutpostToShootBPs(), IntakeCommands.intake(intake)).withTimeout(2.2),
        shoot(ShootingPosition.POSITION_btm).withTimeout(6),
        stop(),
        new InstantCommand(() -> System.out.println("complete routine")));
  }

  public Command StartMidToDepotShoot() {
    return Commands.sequence(
        IntakeCommands.deploy(intake),
        extendKickerbar(),
        new InstantCommand(() -> System.out.println("Moving from mid starting ps to depot")),
        Commands.parallel(goMidToDepot(), IntakeCommands.intake(intake)).withTimeout(3.5),
        new WaitCommand(6),
        new InstantCommand(() -> System.out.println("We are moving to shooting position")),
        Commands.parallel(goDepotToMid(), IntakeCommands.intake(intake)).withTimeout(2.3),
        shoot(ShootingPosition.POSITION_mid).withTimeout(6),
        stop(),
        new InstantCommand(() -> System.out.println("routine complete")));
  }

  public Command StartTopToDepotShoot() {
    return Commands.sequence(
        IntakeCommands.deploy(intake),
        extendKickerbar(),
        new InstantCommand(() -> System.out.println("Moving from top starting ps to depot")),
        Commands.parallel(goTopStartToDepot(), IntakeCommands.intake(intake)).withTimeout(4.3),
        new WaitCommand(6),
        new InstantCommand(() -> System.out.println("We are moving to shooting position")),
        Commands.parallel(goDepotToShootT(), IntakeCommands.intake(intake)).withTimeout(2),
        shoot(ShootingPosition.POSITION_top).withTimeout(6),
        stop(),
        new InstantCommand(() -> System.out.println("routine complete")));
  }

  public Command StartBottomShootOutpost() {
    return Commands.sequence(
        IntakeCommands.deploy(intake),
        extendKickerbar(),
        new InstantCommand(() -> System.out.println("Moving from bottom start to shoot ps")),
        Commands.parallel(goBottomStartToShootB(), IntakeCommands.intake(intake)).withTimeout(1.6),
        shoot(ShootingPosition.POSITION_btm).withTimeout(6),
        new InstantCommand(() -> System.out.println("We are moving to the outpost now")),
        Commands.parallel(goBottomShootertoDepot(), IntakeCommands.intake(intake)).withTimeout(2),
        stop(),
        new InstantCommand(() -> System.out.println("routine complete")));
  }

  public Command StartMidShootDepot() {
    return Commands.sequence(
        IntakeCommands.deploy(intake),
        extendKickerbar(),
        new InstantCommand(() -> System.out.println("Moving from mid start to shooting ps")),
        Commands.parallel(goMidToShooterPs(), IntakeCommands.intake(intake)).withTimeout(1.5),
        shoot(ShootingPosition.POSITION_mid).withTimeout(6),
        new InstantCommand(() -> System.out.println("We are moving to the depot now")),
        Commands.parallel(goMidShootertoDepot(), IntakeCommands.intake(intake)).withTimeout(4),
        stop(),
        new InstantCommand(() -> System.out.println("routine complete")));
  }

  public Command StartTopShootDepot() {
    return Commands.sequence(
        IntakeCommands.deploy(intake),
        extendKickerbar(),
        new InstantCommand(() -> System.out.println("Moving from top start to shooting ps")),
        Commands.parallel(goToptoShooterPs(), IntakeCommands.intake(intake)).withTimeout(1.8),
        shoot(ShootingPosition.POSITION_top).withTimeout(6),
        new InstantCommand(() -> System.out.println("we are moving to depot now")),
        Commands.parallel(goTopShootertoDepot(), IntakeCommands.intake(intake)).withTimeout(4.8),
        stop(),
        new InstantCommand(() -> System.out.println("routine complete")));
  }

  public Command StartBottomNeutralZIntake() {
    return Commands.sequence(
        IntakeCommands.deploy(intake),
        extendKickerbar(),
        new InstantCommand(() -> System.out.println("Moving from bottom start to neutral zone")),
        Commands.parallel(goBottomStartToNeutralZ(), IntakeCommands.intake(intake))
            .withTimeout(8.5),
        // IntakeOnly().withTimeout(3),
        Commands.parallel(goIntakeBtmToAlliance(), IntakeCommands.intake(intake)).withTimeout(7.5),
        outtakeAtAlliance().withTimeout(3),
        stop(),
        new InstantCommand(() -> System.out.println("routine complete")));
  }

  public Command StartTopNeutralZIntake() {
    return Commands.sequence(
        IntakeCommands.deploy(intake),
        extendKickerbar(),
        new InstantCommand(() -> System.out.println("Moving from top start to neutral zone")),
        Commands.parallel(goTopStartToneutralZ(), IntakeCommands.intake(intake)).withTimeout(8),
        // IntakeOnly().withTimeout(3),
        Commands.parallel(goIntakeTopToAlliance(), IntakeCommands.intake(intake)).withTimeout(6),
        outtakeAtAlliance().withTimeout(3),
        stop(),
        new InstantCommand(() -> System.out.println("routine complete")));
  }
}
