package frc.robot.subsystems.auto;

import static frc.robot.subsystems.auto.ChoreoPath.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.IntakeCommands;
import frc.robot.commands.shooter.ShooterCommands;
import frc.robot.commands.shooter.VisionShootCommand;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.hopper.HopperSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.vision.AutoAimUtil;
import frc.robot.subsystems.vision.VisionSubsystem;
import java.util.HashMap;

public class AutoSubsystem extends SubsystemBase {

  private final Drive drive;
  private final ShooterSubsystem shooter;
  // private final ClimberSubsystem climber;
  private final HopperSubsystem hopper;
  private final IntakeSubsystem intake;
  private final VisionSubsystem vision;
  private final HashMap<ChoreoPath, PathPlannerPath> loadedPaths;
  private boolean useVisionShooting = true;

  public AutoSubsystem(
      Drive drive,
      ShooterSubsystem shooter,
      // ClimberSubsystem climber,
      HopperSubsystem hopper,
      IntakeSubsystem intake,
      VisionSubsystem vision) {
    this.drive = drive;
    this.shooter = shooter;
    // this.climber = climber;
    this.hopper = hopper;
    this.intake = intake;
    this.vision = vision;

    loadedPaths = new HashMap<>();
    for (ChoreoPath path : ChoreoPath.values()) {
      try {
        loadedPaths.put(path, PathPlannerPath.fromChoreoTrajectory(path.filepath));
      } catch (Exception e) {
        System.err.println("Failed to load Choreo trajectory: " + path.filepath);
        e.printStackTrace();
      }
    }

    FollowPathCommand.warmupCommand().schedule();
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

  // Auto Actions
  public Command stop() {
    return Commands.runOnce(drive::stop, drive);
  }

  // 40.89397 for bottom shooting ps
  // 31.62039 for top shooting ps
  // 32.74425 for mid shooting ps
  // conditional method
  public Command shoot(ShootingPosition position, double timeSeconds) {
    Command shoot;
    if (useVisionShooting) {
      shoot = new VisionShootCommand(shooter, hopper, drive, vision);
    } else {
      shoot = shootSetpoint(position);
    }

    return Commands.sequence(
        alignToHub(),
        stop(),
        Commands.parallel(
                Commands.sequence(
                    Commands.waitSeconds(timeSeconds - 1), IntakeCommands.runStowed(intake)),
                shoot)
            .withTimeout(timeSeconds));
  }

  public Command alignToHub() {
    return DriveCommands.joystickDriveAtAngle(
            drive, () -> 0, () -> 0, () -> AutoAimUtil.getAngleToHub(drive::getPose))
        .withTimeout(0.25);
  }

  // shooting w setpoint method
  public Command shootSetpoint(ShootingPosition position) {
    return ShooterCommands.shootAtRPS(position.velocity, shooter, hopper);
  }

  public Command deployIntake() {
    System.out.println("Deploying Intake");
    return Commands.sequence(IntakeCommands.deploy(intake), IntakeCommands.moveServoTo90(intake));
  }

  // deadline where intake stops running when traj finishes
  private Command driveAndIntake(Command pathCommand) {
    return Commands.sequence(
        Commands.deadline(pathCommand, IntakeCommands.intake(intake)),
        IntakeCommands.intake(intake).withTimeout(1));
  }

  // loading autoroutines from choreo to pathplanner
  public Command choreoPath(ChoreoPath trajName, boolean resetPose) {
    try {
      // Loading the path from the deploy/choreo folder
      PathPlannerPath path = loadedPaths.get(trajName);
      Command followCommand = AutoBuilder.followPath(path);

      if (resetPose) {
        return new InstantCommand(
                () -> {
                  Pose2d startingPose =
                      DriverStation.getAlliance().orElse(Alliance.Blue) != Alliance.Red
                          ? path.getStartingHolonomicPose().get()
                          : path.flipPath().getStartingHolonomicPose().get();
                  drive.setPose(startingPose, false);
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

  public Command goDepotToShootT() {
    return choreoPath(DepotToShootT, false);
  }

  public Command goMidToDepot() {
    return choreoPath(MidToDepot, true);
  }

  public Command goTopShootertoDepot() {
    return choreoPath(ShootTtoDepot, false);
  }

  public Command goBottomStartToNeutralZ() {
    return choreoPath(bottomStartToneutralZ, true);
  }

  public Command goIntakeBtmToAlliance() {
    return choreoPath(IntakeBtmToAlliance, false);
  }

  public Command goTopStartToneutralZ() {
    return choreoPath(topStartToneutralZ, true);
  }

  public Command goIntakeTopToAlliance() {
    return choreoPath(IntakeToptoAlliance, false);
  }

  public Command intakeNeutralZBtm() {
    return choreoPath(NeutralZBtmIntake, false);
  }

  public Command intakeNeutralZTop() {
    return choreoPath(NeutralZTopIntake, false);
  }

  public Command intakeAtDepot() {
    return choreoPath(DepotIntake, false);
  }

  public Command ShootBtoNeutralZ() {
    return choreoPath(ShootBtoNeutralZ, false);
  }

  public Command TopNeutralZIntakeShootDepot() {
    return Commands.sequence(
        deployIntake(),
        new WaitCommand(0.5),
        goTopStartToneutralZ(),
        stop(),
        driveAndIntake(intakeNeutralZTop()),
        goIntakeTopToAlliance(),
        shoot(ShootingPosition.POSITION_top, 6),
        goTopShootertoDepot(),
        driveAndIntake(intakeAtDepot()),
        stop());
  }

  public Command StartMidToDepotShoot() {
    return Commands.sequence(
        deployIntake(),
        goMidToDepot(),
        stop(),
        driveAndIntake(intakeAtDepot()),
        stop(),
        goDepotToShootT(),
        shoot(ShootingPosition.POSITION_mid, 5),
        stop());
  }

  public Command BottomNeutralZIntakeShootThenNeutralZ() {
    return Commands.sequence(
        deployIntake(),
        Commands.waitSeconds(0.5),
        goBottomStartToNeutralZ(),
        driveAndIntake(intakeNeutralZBtm()),
        goIntakeBtmToAlliance(),
        shoot(ShootingPosition.POSITION_btm, 6),
        ShootBtoNeutralZ(),
        driveAndIntake(intakeNeutralZBtm()),
        goIntakeBtmToAlliance(),
        stop());
  }

  // AUTOROUTINES
  // this routine starts at the bottom start position, moves to a shooting position, and shoots the
  // 8 balls we started with
  //  public Command bottomStartToShootOnly() {
  //    return Commands.sequence(
  //        extendKickerbar(),
  //        new WaitCommand(0.75),
  //        IntakeCommands.deploy(intake),
  //        new WaitCommand(0.7),
  //        retractKickerbar(),
  //        new WaitCommand(0.4),
  //        extendKickerbar(),
  //        new InstantCommand(
  //            () -> System.out.println("Moving from bottom position to Bottom shooting
  // position")),
  //        goBottomStartToShootB(),
  //        new InstantCommand(() -> System.out.println("Reached bottom shooting position")),
  //        shoot(ShootingPosition.POSITION_btm).withTimeout(5),
  //        stop(),
  //        new InstantCommand(() -> System.out.println("routine complete")));
  //  }
  //
  //  // this routine starts at the top start position, moves to a shooting position, and shoots the
  // 8
  //  // balls we started with
  //  public Command topStartToShootOnly() {
  //
  //    return Commands.sequence(
  //        IntakeCommands.deploy(intake),
  //        extendKickerbar(),
  //        new InstantCommand(
  //            () -> System.out.println("Moving from top position to shooting position")),
  //        goToptoShooterPs(),
  //        new InstantCommand(() -> System.out.println("Reached top shooting position")),
  //        shoot(ShootingPosition.POSITION_top).withTimeout(5),
  //        stop(),
  //        new InstantCommand(() -> System.out.println("routine complete")));
  //  }
  //
  //  // this routine starts at the middle start position, moves to a shooting position, and shoots
  // the
  //  // 8 balls we started with
  //  public Command midStartToShootOnly() {
  //    return Commands.sequence(
  //        IntakeCommands.deploy(intake),
  //        new WaitCommand(0.5),
  //        extendKickerbar(),
  //        new InstantCommand(
  //            () -> System.out.println("Moving from mid position to shooting position")),
  //        goMidToShooterPs(),
  //        new InstantCommand(() -> System.out.println("Reached mid shooting position")),
  //        shoot(ShootingPosition.POSITION_mid).withTimeout(5),
  //        stop(),
  //        new InstantCommand(() -> System.out.println("routine complete")));
  //  }
  //  // this routine starts at the bottom starting position, and moves directly to the outppst and
  //  // waits for 3s,human
  //  // player opens it, robot holds more fuel,
  //  // and then the robot moves to a shooting position and shoots
  //  public Command StartBottomToOutpostShoot() {
  //    return Commands.sequence(
  //        IntakeCommands.deploy(intake),
  //        extendKickerbar(),
  //        new InstantCommand(() -> System.out.println("Moving from bottom start ps to outpost")),
  //        goBottomToOutpost(),
  //        new WaitCommand(3),
  //        new InstantCommand(() -> System.out.println("Moving to Shooter B Position")),
  //        goOutpostToShootBPs(),
  //        shoot(ShootingPosition.POSITION_btm).withTimeout(5),
  //        stop(),
  //        new InstantCommand(() -> System.out.println("complete routine")));
  //  }

  // this routine starts at the middle starting position, and moves directly to the depot,intakes
  // fuel,
  // and then the robot moves to a shooting position and shoots

  //  // this routine starts at the top starting position, and moves directly to the depot,intakes
  // fuel,
  //  // and then the robot moves to a shooting position and shoots
  //  public Command StartTopToDepotShoot() {
  //    return Commands.sequence(
  //        IntakeCommands.deploy(intake),
  //        extendKickerbar(),
  //        new InstantCommand(() -> System.out.println("Moving from top starting ps to depot")),
  //        goTopStartToDepot(),
  //        driveAndIntake(intakeAtDepot()),
  //        new InstantCommand(() -> System.out.println("We are moving to shooting position")),
  //        goDepotToShootT(),
  //        shoot(ShootingPosition.POSITION_top).withTimeout(5),
  //        stop(),
  //        new InstantCommand(() -> System.out.println("routine complete")));
  //  }
  //
  //  // this routine starts at the bottom starting position, moves to a shooting position, shoots,
  // and
  //  // then goes to the outpost to fuel up
  //  public Command StartBottomShootOutpost() {
  //    return Commands.sequence(
  //        IntakeCommands.deploy(intake),
  //        extendKickerbar(),
  //        new InstantCommand(() -> System.out.println("Moving from bottom start to shoot ps")),
  //        goBottomStartToShootB(),
  //        shoot(ShootingPosition.POSITION_btm).withTimeout(5),
  //        new InstantCommand(() -> System.out.println("We are moving to the outpost now")),
  //        goBottomShootertoDepot(),
  //        stop(),
  //        new InstantCommand(() -> System.out.println("routine complete")));
  //  }
  //
  //  // this routine starts at the middle starting position, moves to a shooting position, shoots,
  // and
  //  // then goes to the depot to fuel up
  //  public Command StartMidShootDepot() {
  //    return Commands.sequence(
  //        IntakeCommands.deploy(intake),
  //        extendKickerbar(),
  //        new InstantCommand(() -> System.out.println("Moving from mid start to shooting ps")),
  //        goMidToShooterPs(),
  //        shoot(ShootingPosition.POSITION_mid).withTimeout(5),
  //        new InstantCommand(() -> System.out.println("We are moving to the depot now")),
  //        goMidShootertoDepot(),
  //        driveAndIntake(intakeAtDepot()),
  //        IntakeCommands.intake(intake).withTimeout(2),
  //        stop(),
  //        new InstantCommand(() -> System.out.println("routine complete")));
  //  }
  //  // this routine starts at the top starting position, moves to a shooting position, shoots, and
  //  // then goes to the depot to fuel up
  //  public Command StartTopShootDepot() {
  //    return Commands.sequence(
  //        IntakeCommands.deploy(intake),
  //        extendKickerbar(),
  //        new InstantCommand(() -> System.out.println("Moving from top start to shooting ps")),
  //        goToptoShooterPs(),
  //        shoot(ShootingPosition.POSITION_top).withTimeout(5),
  //        new InstantCommand(() -> System.out.println("we are moving to depot now")),
  //        goTopShootertoDepot(),
  //        driveAndIntake(intakeAtDepot()),
  //        stop(),
  //        new InstantCommand(() -> System.out.println("routine complete")));
  //  }
  //  // robot starts at the bottom starting position, goes to the neutral zone (the Z stands for
  // zone
  //  // smh), and then intakes fuel
  //  // after intaking, the robot comes back into the alliance zone, and outtakes the balls
  //  public Command StartBottomNeutralZIntake() {
  //    return Commands.sequence(
  //        IntakeCommands.deploy(intake),
  //        extendKickerbar(),
  //        new InstantCommand(() -> System.out.println("Moving from bottom start to neutral
  // zone")),
  //        goBottomStartToNeutralZ(),
  //        driveAndIntake(intakeNeutralZBtm()),
  //        goIntakeBtmToAlliance(),
  //        outtakeAtAlliance().withTimeout(3),
  //        stop(),
  //        new InstantCommand(() -> System.out.println("routine complete")));
  //  }
  //
  //  // robot starts at the top starting position, goes to the neutral zone (the Z stands for zone
  //  // smh), and then intakes fuel
  //  // after intaking, the robot comes back into the alliance zone, and outtakes the balls
  //  public Command StartTopNeutralZIntake() {
  //    return Commands.sequence(
  //        IntakeCommands.deploy(intake),
  //        extendKickerbar(),
  //        new InstantCommand(() -> System.out.println("Moving from top start to neutral zone")),
  //        goTopStartToneutralZ(),
  //        driveAndIntake(intakeNeutralZTop()),
  //        goIntakeTopToAlliance(),
  //        outtakeAtAlliance().withTimeout(3),
  //        stop(),
  //        new InstantCommand(() -> System.out.println("routine complete")));
  //  }
  //
  //  // robot starts at the top starting position, goes to the neutral zone (the Z stands for zone
  //  // smh), and then intakes fuel
  //  // after intaking, the robot turns towards our alliance and shoots balls towards our zone
  //  public Command StartTopNeutralZDump() {
  //    return Commands.sequence(
  //        IntakeCommands.deploy(intake),
  //        extendKickerbar(),
  //        new InstantCommand(() -> System.out.println("Moving from top start to neutral zone")),
  //        goTopStartToneutralZ(),
  //        driveAndIntake(intakeNeutralZTop()),
  //        NeutralZAimAllianceTop(),
  //        new InstantCommand(() -> System.out.println("We are now shooting towards alliance")),
  //        shoot(ShootingPosition.POSITION_top).withTimeout(12),
  //        stop(),
  //        new InstantCommand(() -> System.out.println("routine complete")));
  //  }
  //
  //  // robot starts at the bottom starting position, goes to the neutral zone (the Z stands for
  // zone
  //  // smh), and then intakes fuel
  //  // after intaking, the robot turns towards our alliance and shoots balls towards our zone
  //  public Command StartBottomNeutralZDump() {
  //    return Commands.sequence(
  //        Commands.sequence(
  //            extendKickerbar(), Commands.waitSeconds(0.1), IntakeCommands.deploy(intake)),
  //        new InstantCommand(() -> System.out.println("Moving from top start to neutral zone")),
  //        goBottomStartToNeutralZ(),
  //        driveAndIntake(intakeNeutralZBtm()),
  //        NeutralZAimAllianceBtm(),
  //        new InstantCommand(() -> System.out.println("We are now shooting towards alliance")),
  //        shoot(ShootingPosition.POSITION_btm).withTimeout(12),
  //        stop(),
  //        new InstantCommand(() -> System.out.println("routine complete")));
  //  }
  //
  //  // below is the same code for a diff routine so its commented out for now
  //  /*public Command StartTopNeutralZShootThenIntake() {
  //    return Commands.sequence(
  //        IntakeCommands.deploy(intake),
  //        extendKickerbar(),
  //        new InstantCommand(() -> System.out.println("Moving from top start to neutral zone")),
  //        goTopStartToneutralZ(),
  //        driveAndIntake(intakeNeutralZTop()),
  //        NeutralZAimAllianceTop(),
  //        new InstantCommand(() -> System.out.println("We are now shooting towards alliance")),
  //        shoot(ShootingPosition.POSITION_top).withTimeout(12),
  //        stop(),
  //        new InstantCommand(() -> System.out.println("routine complete")));
  //  }
  //  */
  //  public Command StartBottomNeutralZthenShootBottom() {
  //    return Commands.sequence(
  //        IntakeCommands.deploy(intake),
  //        extendKickerbar(),
  //        new InstantCommand(() -> System.out.println("Moving from bottom start to neutral zone
  // ")),
  //        goBottomStartToNeutralZ(),
  //        driveAndIntake(intakeNeutralZBtm()),
  //        goIntakeBtmToAlliance(),
  //        OuttakePstoShootB(),
  //        shoot(ShootingPosition.POSITION_btm).withTimeout(12),
  //        stop(),
  //        new InstantCommand(() -> System.out.println("routine complete")));
  //  }
  //
  //  public Command StartBottomNeutralZthenShootTop() {
  //    return Commands.sequence(
  //        IntakeCommands.deploy(intake),
  //        extendKickerbar(),
  //        new InstantCommand(() -> System.out.println("Moving from bottom start to neutral zone
  // ")),
  //        goTopStartToneutralZ(),
  //        driveAndIntake(intakeNeutralZTop()),
  //        goIntakeTopToAlliance(),
  //        OuttakePstoShootT(),
  //        shoot(ShootingPosition.POSITION_top).withTimeout(12),
  //        stop(),
  //        new InstantCommand(() -> System.out.println("routine complete")));
  //  }

}
