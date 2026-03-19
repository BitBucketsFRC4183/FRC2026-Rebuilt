package frc.robot.subsystems.auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.commands.ClimberCommands;
import frc.robot.commands.IntakeCommands;
import frc.robot.commands.shooter.ShooterCommands;
import frc.robot.commands.shooter.VisionShootCommand;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.hopper.HopperSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;

public class AutoSubsystem extends SubsystemBase {

  private final Drive drive;
  private final ShooterSubsystem shooter;
  private final ClimberSubsystem climber;
  private final HopperSubsystem hopper;
  private final IntakeSubsystem intake;
  private final VisionSubsystem vision;
  private boolean useVisionShooting = true;

  public AutoSubsystem(
      Drive drive,
      ShooterSubsystem shooter,
      ClimberSubsystem climber,
      HopperSubsystem hopper,
      IntakeSubsystem intake,
      VisionSubsystem vision) {
    this.drive = drive;
    this.shooter = shooter;
    this.climber = climber;
    this.hopper = hopper;
    this.intake = intake;
    this.vision = vision;
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

  // 40.89397 for bottom shooting ps
  // 31.62039 for top shooting ps
  // 32.74425 for mid shooting ps
  public void setUseVisionShooting(boolean useVision) {
    this.useVisionShooting = useVision;
  }

  // conditional method
  public Command shoot(ShootingPosition position) {
    if (useVisionShooting) {
      System.out.println("shooting with vision");
      return new VisionShootCommand(shooter, hopper, drive, vision);
    } else {
      System.out.println("shooting with setpoint");
      return shootSetpoint(position);
    }
  }
  // shooting w setpoint method
  public Command shootSetpoint(ShootingPosition position) {
    System.out.println("Beginning to Shoot with Setpoint");
    return ShooterCommands.shootAtRPS(position.velocity, shooter, hopper);
  }

  public Command stopshooter(ShootingPosition position) {
    System.out.println("Beginning to Shoot");
    return ShooterCommands.reset(shooter, hopper);
  }

  public Command climb() {
    return ClimberCommands.increaseClimberLengthLevelOne(climber)
        .beforeStarting(() -> System.out.println("Climbing!"));
  }

  public Command extendKickerbar() {
    System.out.println("extending kickerbar");
    return IntakeCommands.moveServoTo90(intake);
  }

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

  // deadline where intake stops running when traj finishes
  private Command driveAndIntake(Command pathCommand) {
    return Commands.deadline(pathCommand, IntakeCommands.intake(intake));
  }

  // loading autoroutines from choreo to pathplanner
  public Command choreoPath(String trajName, boolean resetPose) {
    try {
      // Loading the path from the deploy/choreo folder
      PathPlannerPath path = PathPlannerPath.fromChoreoTrajectory(trajName);
      Command followCommand = AutoBuilder.followPath(path);

      if (resetPose) {
        return new InstantCommand(
                () -> {
                  Pose2d startingPose =
                      DriverStation.getAlliance().orElse(Alliance.Blue) != Alliance.Red
                          ? path.getStartingHolonomicPose().get()
                          : path.flipPath().getStartingHolonomicPose().get();
                  drive.setPose(startingPose);
                })
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

  public Command goBottomShootertoDepot() {
    return choreoPath("ShootBtoDepot", false);
  }

  public Command goMidShootertoDepot() {
    return choreoPath("ShootMtoDepot", false);
  }

  public Command goDepotToMid() {
    return choreoPath("DepotToMid", false);
  }

  public Command goBottomToOutpost() {
    return choreoPath("BottomToOutpost", true);
  }

  public Command goOutpostToShootBPs() {
    return choreoPath("OutpostToShooterB", false);
  }

  public Command goMidToDepot() {
    return choreoPath("MidToDepot", true);
  }

  public Command goTopStartToDepot() {
    return choreoPath("TopStartToDepot", true);
  }

  public Command goDepotToShootT() {
    return choreoPath("DepotToShootT", false);
  }

  public Command goBottomStartToShootB() {
    return choreoPath("BottomStartToShootB", false);
  }

  public Command goTopShootertoDepot() {
    return choreoPath("ShootTtoDepot", false);
  }

  public Command goBottomStartToNeutralZ() {
    return choreoPath("bottomStartToneutralZ", true);
  }

  public Command goIntakeBtmToAlliance() {
    return choreoPath("IntakeBtmToAlliance", false);
  }

  public Command goTopStartToneutralZ() {
    return choreoPath("topStartToneutralZ", true);
  }

  public Command goIntakeTopToAlliance() {
    return choreoPath("IntakeToptoAlliance", false);
  }

  public Command intakeNeutralZBtm() {
    return choreoPath("NeutralZBtmIntake", false);
  }

  public Command intakeNeutralZTop() {
    return choreoPath("NeutralZTopIntake", false);
  }

  public Command intakeAtDepot() {
    return choreoPath("DepotIntake", false);
  }

  public Command NeutralZAimAllianceTop() {
    return choreoPath("NeutralZAimAllianceTop", false);
  }

  public Command NeutralZAimAllianceBtm() {
    return choreoPath("NeutralZAimAllianceBtm", false);
  }

  // AUTOROUTINES
  // this routine starts at the bottom start position, moves to a shooting position, and shoots the
  // 8 balls we started with
  public Command bottomStartToShootOnly() {
    return Commands.sequence(
        IntakeCommands.deploy(intake),
        extendKickerbar(),
        new InstantCommand(
            () -> System.out.println("Moving from bottom position to Bottom shooting position")),
        goBottomStartToShootB(),
        new InstantCommand(() -> System.out.println("Reached bottom shooting position")),
        shoot(ShootingPosition.POSITION_btm).withTimeout(5),
        stop(),
        new InstantCommand(() -> System.out.println("routine complete")));
  }

  // this routine starts at the top start position, moves to a shooting position, and shoots the 8
  // balls we started with
  public Command topStartToShootOnly() {

    return Commands.sequence(
        IntakeCommands.deploy(intake),
        extendKickerbar(),
        new InstantCommand(
            () -> System.out.println("Moving from top position to shooting position")),
        goToptoShooterPs(),
        new InstantCommand(() -> System.out.println("Reached top shooting position")),
        shoot(ShootingPosition.POSITION_top).withTimeout(5),
        stop(),
        new InstantCommand(() -> System.out.println("routine complete")));
  }

  // this routine starts at the middle start position, moves to a shooting position, and shoots the
  // 8 balls we started with
  public Command midStartToShootOnly() {
    return Commands.sequence(
        IntakeCommands.deploy(intake),
        extendKickerbar(),
        new InstantCommand(
            () -> System.out.println("Moving from mid position to shooting position")),
        goMidToShooterPs(),
        new InstantCommand(() -> System.out.println("Reached mid shooting position")),
        shoot(ShootingPosition.POSITION_mid).withTimeout(5),
        stop(),
        new InstantCommand(() -> System.out.println("routine complete")));
  }
  // this routine starts at the bottom starting position, and moves directly to the outppst and
  // waits for 3s,human
  // player opens it, robot holds more fuel,
  // and then the robot moves to a shooting position and shoots
  public Command StartBottomToOutpostShoot() {
    return Commands.sequence(
        IntakeCommands.deploy(intake),
        extendKickerbar(),
        new InstantCommand(() -> System.out.println("Moving from bottom start ps to outpost")),
        goBottomToOutpost(),
        new WaitCommand(3),
        new InstantCommand(() -> System.out.println("Moving to Shooter B Position")),
        goOutpostToShootBPs(),
        shoot(ShootingPosition.POSITION_btm).withTimeout(5),
        stop(),
        new InstantCommand(() -> System.out.println("complete routine")));
  }

  // this routine starts at the middle starting position, and moves directly to the depot,intakes
  // fuel,
  // and then the robot moves to a shooting position and shoots
  public Command StartMidToDepotShoot() {
    return Commands.sequence(
        IntakeCommands.deploy(intake),
        extendKickerbar(),
        new InstantCommand(() -> System.out.println("Moving from mid starting ps to depot")),
        goMidToDepot(),
        driveAndIntake(intakeAtDepot()),
        new InstantCommand(() -> System.out.println("We are moving to shooting position")),
        goDepotToMid(),
        shoot(ShootingPosition.POSITION_mid).withTimeout(5),
        stop(),
        new InstantCommand(() -> System.out.println("routine complete")));
  }

  // this routine starts at the top starting position, and moves directly to the depot,intakes fuel,
  // and then the robot moves to a shooting position and shoots
  public Command StartTopToDepotShoot() {
    return Commands.sequence(
        IntakeCommands.deploy(intake),
        extendKickerbar(),
        new InstantCommand(() -> System.out.println("Moving from top starting ps to depot")),
        goTopStartToDepot(),
        driveAndIntake(intakeAtDepot()),
        new InstantCommand(() -> System.out.println("We are moving to shooting position")),
        goDepotToShootT(),
        shoot(ShootingPosition.POSITION_top).withTimeout(5),
        stop(),
        new InstantCommand(() -> System.out.println("routine complete")));
  }

  // this routine starts at the bottom starting position, moves to a shooting position, shoots, and
  // then goes to the outpost to fuel up
  public Command StartBottomShootOutpost() {
    return Commands.sequence(
        IntakeCommands.deploy(intake),
        extendKickerbar(),
        new InstantCommand(() -> System.out.println("Moving from bottom start to shoot ps")),
        goBottomStartToShootB(),
        shoot(ShootingPosition.POSITION_btm).withTimeout(5),
        new InstantCommand(() -> System.out.println("We are moving to the outpost now")),
        goBottomShootertoDepot(),
        stop(),
        new InstantCommand(() -> System.out.println("routine complete")));
  }

  // this routine starts at the middle starting position, moves to a shooting position, shoots, and
  // then goes to the depot to fuel up
  public Command StartMidShootDepot() {
    return Commands.sequence(
        IntakeCommands.deploy(intake),
        extendKickerbar(),
        new InstantCommand(() -> System.out.println("Moving from mid start to shooting ps")),
        goMidToShooterPs(),
        shoot(ShootingPosition.POSITION_mid).withTimeout(5),
        new InstantCommand(() -> System.out.println("We are moving to the depot now")),
        goMidShootertoDepot(),
        driveAndIntake(intakeAtDepot()),
        stop(),
        new InstantCommand(() -> System.out.println("routine complete")));
  }
  // this routine starts at the top starting position, moves to a shooting position, shoots, and
  // then goes to the depot to fuel up
  public Command StartTopShootDepot() {
    return Commands.sequence(
        IntakeCommands.deploy(intake),
        extendKickerbar(),
        new InstantCommand(() -> System.out.println("Moving from top start to shooting ps")),
        goToptoShooterPs(),
        shoot(ShootingPosition.POSITION_top).withTimeout(5),
        driveAndIntake(intakeAtDepot()),
        new InstantCommand(() -> System.out.println("we are moving to depot now")),
        goTopShootertoDepot(),
        stop(),
        new InstantCommand(() -> System.out.println("routine complete")));
  }
  // robot starts at the bottom starting position, goes to the neutral zone (the Z stands for zone
  // smh), and then intakes fuel
  // after intaking, the robot comes back into the alliance zone, and outtakes the balls
  public Command StartBottomNeutralZIntake() {
    return Commands.sequence(
        IntakeCommands.deploy(intake),
        extendKickerbar(),
        new InstantCommand(() -> System.out.println("Moving from bottom start to neutral zone")),
        goBottomStartToNeutralZ(),
        driveAndIntake(intakeNeutralZBtm()),
        goIntakeBtmToAlliance(),
        outtakeAtAlliance().withTimeout(3),
        stop(),
        new InstantCommand(() -> System.out.println("routine complete")));
  }

  // robot starts at the top starting position, goes to the neutral zone (the Z stands for zone
  // smh), and then intakes fuel
  // after intaking, the robot comes back into the alliance zone, and outtakes the balls
  public Command StartTopNeutralZIntake() {
    return Commands.sequence(
        IntakeCommands.deploy(intake),
        extendKickerbar(),
        new InstantCommand(() -> System.out.println("Moving from top start to neutral zone")),
        goTopStartToneutralZ(),
        driveAndIntake(intakeNeutralZTop()),
        goIntakeTopToAlliance(),
        outtakeAtAlliance().withTimeout(3),
        stop(),
        new InstantCommand(() -> System.out.println("routine complete")));
  }

  // robot starts at the top starting position, goes to the neutral zone (the Z stands for zone
  // smh), and then intakes fuel
  // after intaking, the robot turns towards our alliance and shoots balls towards our zone
  public Command StartTopNeutralZDump() {
    return Commands.sequence(
        IntakeCommands.deploy(intake),
        extendKickerbar(),
        new InstantCommand(() -> System.out.println("Moving from top start to neutral zone")),
        goTopStartToneutralZ(),
        driveAndIntake(intakeNeutralZTop()),
        NeutralZAimAllianceTop(),
        new InstantCommand(() -> System.out.println("We are now shooting towards alliance")),
        shoot(ShootingPosition.POSITION_btm).withTimeout(12),
        stop(),
        new InstantCommand(() -> System.out.println("routine complete")));
  }

  // robot starts at the bottom starting position, goes to the neutral zone (the Z stands for zone
  // smh), and then intakes fuel
  // after intaking, the robot turns towards our alliance and shoots balls towards our zone
  public Command StartBottomNeutralZDump() {
    return Commands.sequence(
        IntakeCommands.deploy(intake),
        extendKickerbar(),
        new InstantCommand(() -> System.out.println("Moving from top start to neutral zone")),
        goTopStartToneutralZ(),
        driveAndIntake(intakeNeutralZTop()),
        NeutralZAimAllianceBtm(),
        new InstantCommand(() -> System.out.println("We are now shooting towards alliance")),
        shoot(ShootingPosition.POSITION_btm).withTimeout(12),
        stop(),
        new InstantCommand(() -> System.out.println("routine complete")));
  }

  public Command StartTopNeutralZShootThenIntake() {
    return Commands.sequence(
        IntakeCommands.deploy(intake),
        extendKickerbar(),
        new InstantCommand(() -> System.out.println("Moving from top start to neutral zone")),
        goTopStartToneutralZ(),
        driveAndIntake(intakeNeutralZTop()),
        NeutralZAimAllianceTop(),
        new InstantCommand(() -> System.out.println("We are now shooting towards alliance")),
        shoot(ShootingPosition.POSITION_btm).withTimeout(12),
        stop(),
        new InstantCommand(() -> System.out.println("routine complete")));
  }
}
