// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.*;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.auto.AutoSubsystem;
import frc.robot.subsystems.climber.ClimberIO;
import frc.robot.subsystems.climber.ClimberIOSim;
import frc.robot.subsystems.climber.ClimberIOTalonFX;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.hopper.*;
import frc.robot.subsystems.intake.*;
import frc.robot.subsystems.power_distribution.PowerDistributionSubsystem;
import frc.robot.subsystems.shooter.*;
import frc.robot.subsystems.vision.*;
import java.util.function.Supplier;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer {
  // Subsystems
  private final Drive driveSubsystem;
  private final HopperSubsystem hopperSubsystem;
  private final ShooterSubsystem shooterSubsystem;
  private final IntakeSubsystem intakeSubsystem;
  private PowerDistributionSubsystem powerSubsystem;
  private final Field2d field;
  private AutoSubsystem autoSubsystem;
  private final LoggedDashboardChooser<Command> autoChooser;

  private VisionSubsystem visionSubsystem;
  private VisionIO visionIO;

  // Added missing subsystem fields
  private ClimberIO climberIO;
  private ClimberSubsystem climberSubsystem;

  // Controller
  private final CommandXboxController driverController = new CommandXboxController(0);
  private final CommandXboxController operatorController = new CommandXboxController(1);

  private SwerveDriveSimulation driveSimulation = null;

  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        driveSubsystem =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFXAnalog(TunerConstants.FrontLeft),
                new ModuleIOTalonFXAnalog(TunerConstants.FrontRight),
                new ModuleIOTalonFXAnalog(TunerConstants.BackLeft),
                new ModuleIOTalonFXAnalog(TunerConstants.BackRight),
                (pose) -> {});

        visionSubsystem =
            new VisionSubsystem(
                new VisionIOLimelight(),
                () -> driveSubsystem.poseEstimator.getEstimatedPosition(),
                driveSubsystem);

        climberSubsystem = new ClimberSubsystem(new ClimberIOTalonFX());
        intakeSubsystem = new IntakeSubsystem(new IntakeIOTalonFX());
        shooterSubsystem = new ShooterSubsystem(new ShooterIOTalonFX());
        hopperSubsystem = new HopperSubsystem(new HopperIOTalonFX());
        powerSubsystem = new PowerDistributionSubsystem(intakeSubsystem, shooterSubsystem);

        driveSubsystem.setLimelightIMUCallback = (rot) -> visionSubsystem.setLimelightIMUGyro(rot);
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        driveSimulation =
            new SwerveDriveSimulation(Drive.mapleSimConfig, new Pose2d(3, 3, new Rotation2d(0, 0)));
        SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation);

        driveSubsystem =
            new Drive(
                new GyroIOSim(driveSimulation.getGyroSimulation()),
                new ModuleIOTalonFXSim(TunerConstants.FrontLeft, driveSimulation.getModules()[0]),
                new ModuleIOTalonFXSim(TunerConstants.FrontRight, driveSimulation.getModules()[1]),
                new ModuleIOTalonFXSim(TunerConstants.BackLeft, driveSimulation.getModules()[2]),
                new ModuleIOTalonFXSim(TunerConstants.BackRight, driveSimulation.getModules()[3]),
                driveSimulation::setSimulationWorldPose);

        driveSubsystem.setPose(new Pose2d(3, 3, new Rotation2d()));

        visionSubsystem =
            new VisionSubsystem(
                new VisionIOPhotonVisionSim(() -> driveSimulation.getSimulatedDriveTrainPose()),
                () -> driveSimulation.getSimulatedDriveTrainPose(),
                driveSubsystem);

        climberIO = new ClimberIOSim();
        climberSubsystem = new ClimberSubsystem(climberIO);
        intakeSubsystem = new IntakeSubsystem(new IntakeIOSim());
        shooterSubsystem = new ShooterSubsystem(new ShooterIOTalonFX());
        hopperSubsystem = new HopperSubsystem(new HopperIOTalonFX());
        powerSubsystem = new PowerDistributionSubsystem(intakeSubsystem, shooterSubsystem);
        break;
        // thinking to what

      default:
        // Replayed robot, disable IO implementations
        driveSubsystem =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                (pose) -> {});

        climberSubsystem = new ClimberSubsystem(new ClimberIO() {});
        intakeSubsystem = new IntakeSubsystem(new IntakeIO() {});
        shooterSubsystem = new ShooterSubsystem(new ShooterIO() {});
        hopperSubsystem = new HopperSubsystem(new HopperIO() {});
        powerSubsystem = new PowerDistributionSubsystem(intakeSubsystem, shooterSubsystem);
        visionSubsystem =
            new VisionSubsystem(
                visionIO, () -> driveSimulation.getSimulatedDriveTrainPose(), driveSubsystem);

        break;
    }

    autoSubsystem =
        new AutoSubsystem(
            driveSubsystem, shooterSubsystem, climberSubsystem, hopperSubsystem, intakeSubsystem);

    // WARMUP commands
    FollowPathCommand.warmupCommand().schedule();
    // PathfindingCommand.warmupCommand().schedule();
    // Set up auto routines
    var chooser = AutoBuilder.buildAutoChooser();
    autoChooser = new LoggedDashboardChooser<>("/SmartDashboard/Auto Chooser", chooser);

    // putting chooser on dashboard
    SmartDashboard.putData("Auto Chooser", autoChooser.getSendableChooser());

    // Set up SysId routines
    //    autoChooser.addOption(
    //        "DriveSubsystem Wheel Radius Characterization",
    //        DriveCommands.wheelRadiusCharacterization(driveSubsystem));

    //    autoChooser.addOption(
    //        "DriveSubsystem SysId (Quasistatic Forward)",
    //        driveSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    //    autoChooser.addOption(
    //        "DriveSubsystem SysId (Quasistatic Reverse)",
    //        driveSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    //    autoChooser.addOption(
    //        "DriveSubsystem SysId (Dynamic Forward)",
    //        driveSubsystem.sysIdDynamic(SysIdRoutine.Direction.kForward));
    //    autoChooser.addOption(
    //        "DriveSubsystem SysId (Dynamic Reverse)",
    //        driveSubsystem.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    autoChooser.addOption("bottomStartToShootOnly", autoSubsystem.bottomStartToShootOnly());
    autoChooser.addOption("topStartToShootOnly", autoSubsystem.topStartToShootOnly());
    autoChooser.addOption("midStartToShootOnly", autoSubsystem.midStartToShootOnly());
    autoChooser.addOption("StartBottomToOutpostShoot", autoSubsystem.StartBottomToOutpostShoot());
    autoChooser.addOption("StartMidToDepotShoot", autoSubsystem.StartMidToDepotShoot());
    autoChooser.addOption("StartTopToDepotShoot", autoSubsystem.StartTopToDepotShoot());
    autoChooser.addOption("StartBottomShootOutpost", autoSubsystem.StartBottomShootOutpost());
    autoChooser.addOption(" StartMidShootDepot", autoSubsystem.StartMidShootDepot());
    autoChooser.addOption("StartTopShootDepot", autoSubsystem.StartTopShootDepot());
    autoChooser.addOption("StartBottomNeutralZIntake", autoSubsystem.StartBottomNeutralZIntake());
    autoChooser.addOption("StartTopNeutralZIntake", autoSubsystem.StartTopNeutralZIntake());

    autoChooser.addOption(
        "ShooterSubsystem SysId (Quasistatic Forward)",
        shooterSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "ShooterSubsystem SysId (Quasistatic Reverse)",
        shooterSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "ShooterSubsystem SysId (Dynamic Forward)",
        shooterSubsystem.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "ShooterSubsystem SysId (Dynamic Reverse)",
        shooterSubsystem.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Configure the button bindings
    configureButtonBindings();
    field = new Field2d();
    SmartDashboard.putData(field);
  }

  public void robotPeriodic() {
    field.setRobotPose(driveSubsystem.getPose());
  }

  private void configureButtonBindings() {
    // Default command, normal field-relative driveSubsystem
    driveSubsystem.setDefaultCommand(
        DriveCommands.joystickDrive(
            driveSubsystem,
            powerSubsystem,
            () -> -driverController.getLeftY(),
            () -> -driverController.getLeftX(),
            () -> -driverController.getRightX()));

    driverController.povUp().whileTrue(driverJoystickDriveAtAngle(() -> Rotation2d.kZero));
    driverController.povRight().whileTrue(driverJoystickDriveAtAngle(() -> Rotation2d.kCW_90deg));
    driverController.povDown().whileTrue(driverJoystickDriveAtAngle(() -> Rotation2d.k180deg));
    driverController.povLeft().whileTrue(driverJoystickDriveAtAngle(() -> Rotation2d.kCCW_90deg));

    // Switch to X pattern when X button is pressed
    driverController.x().onTrue(Commands.runOnce(driveSubsystem::stopWithX, driveSubsystem));

    driverController
        .a()
        .whileTrue(
            driverJoystickDriveAtAngle(() -> AutoAimUtil.getAngletoHub(driveSubsystem.getPose())));

    // temp only
    // driverController.leftBumper().onTrue(Commands.runOnce(SignalLogger::start));
    // driverController.rightBumper().onTrue(Commands.runOnce(SignalLogger::stop));

    // Reset gyro / odometry
    final Runnable resetOdometry =
        Constants.currentMode == Constants.Mode.SIM
            ? () -> resetSimulation(new Pose2d(3, 3, new Rotation2d()))
            : () ->
                driveSubsystem.setPose(
                    new Pose2d(driveSubsystem.getPose().getTranslation(), new Rotation2d()));
    driverController.start().onTrue(Commands.runOnce(resetOdometry).ignoringDisable(true));

    // Overrides the DPD Subsystem
    driverController
        .back()
        .onTrue(Commands.runOnce(() -> powerSubsystem.setOverride(true)))
        .onFalse(Commands.runOnce(() -> powerSubsystem.setOverride(false)));

    operatorController
        .leftBumper()
        .onTrue(
            Commands.runOnce(
                () -> {
                  if (intakeSubsystem.getState() == IntakeState.STOWED) {
                    intakeSubsystem.deploy();
                  } else {
                    intakeSubsystem.stow();
                  }
                },
                intakeSubsystem));

    operatorController.leftTrigger().whileTrue(IntakeCommands.intake(intakeSubsystem));
    operatorController.povLeft().whileTrue(IntakeCommands.setServoPulseNegative(intakeSubsystem));
    operatorController.povRight().whileTrue(IntakeCommands.setServoPulsePositive(intakeSubsystem));

    // Hopper runs, will change to intake later
    operatorController
        .rightBumper()
        .whileTrue(
            Commands.startEnd(
                hopperSubsystem::runConveyorForward,
                hopperSubsystem::stopConveyor,
                hopperSubsystem));

    operatorController
        .rightTrigger()
        .whileTrue(
            ShooterCommands.visionShoot(
                visionSubsystem.getHubDistanceMeter(driveSubsystem.getPose()),
                shooterSubsystem,
                hopperSubsystem))
        .onFalse(ShooterCommands.reset(shooterSubsystem, hopperSubsystem));

    operatorController.b().whileTrue(IntakeCommands.outtake(intakeSubsystem));
    //     Climber Setpoint Commands
    operatorController
        .x()
        .and(operatorController.back())
        .onTrue(ClimberCommands.climbToLevelOne(climberSubsystem, driveSubsystem));

    operatorController.b().whileTrue(IntakeCommands.outtake(intakeSubsystem));

    // manual climb command
    new Trigger(
            () ->
                Math.abs(operatorController.getLeftY()) > 0.1
                    && (operatorController.povDown()).getAsBoolean())
        .whileTrue(ClimberCommands.joystickClimb(climberSubsystem, operatorController::getLeftY));
  }

  public Command driverJoystickDriveAtAngle(Supplier<Rotation2d> rotation) {
    return DriveCommands.joystickDriveAtAngle(
        driveSubsystem,
        () -> -driverController.getLeftY(),
        () -> -driverController.getLeftX(),
        rotation);
  }

  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  public void resetSimulation(Pose2d newPose) {
    if (Constants.currentMode != Constants.Mode.SIM) return;

    driveSimulation.setSimulationWorldPose(newPose);
    SimulatedArena.getInstance().resetFieldForAuto();
  }

  public void updateSimulation() {
    if (Constants.currentMode != Constants.Mode.SIM) return;

    SimulatedArena.getInstance().simulationPeriodic();
    Logger.recordOutput(
        "FieldSimulation/RobotPosition", driveSimulation.getSimulatedDriveTrainPose());
  }
}
