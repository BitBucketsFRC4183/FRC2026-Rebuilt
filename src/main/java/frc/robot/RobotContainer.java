// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.*;
import frc.robot.constants.VisionConstant;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.auto.AutoSubsystem;
import frc.robot.subsystems.climber.ClimberIO;
import frc.robot.subsystems.climber.ClimberIOSim;
import frc.robot.subsystems.climber.ClimberIOTalonFX;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.hopper.*;
import frc.robot.subsystems.intake.*;
import frc.robot.subsystems.shooter.*;
import frc.robot.subsystems.vision.*;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.Logger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive driveSubsystem;
  private final HopperSubsystem hopperSubsystem;
  private final ShooterSubsystem shooterSubsystem;
  private final IntakeSubsystem intakeSubsystem;

  private AutoSubsystem autoSubsystem;
  private final SendableChooser<Command> autoChooser;

  private VisionSubsystem visionSubsystem;
  private VisionIO visionIO;
  private OdometryHistory odometryHistory;
  private VisionFusionResults visionFusionResults;

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
        // Real robot, instantiate hardware IO implementations
        // ModuleIOTalonFX is intended for modules with TalonFX driveSubsystem, TalonFX turn, and
        // a CANcoder
        driveSubsystem =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFXAnalog(TunerConstants.FrontLeft),
                new ModuleIOTalonFXAnalog(TunerConstants.FrontRight),
                new ModuleIOTalonFXAnalog(TunerConstants.BackLeft),
                new ModuleIOTalonFXAnalog(TunerConstants.BackRight),
                (pose) -> {});

        climberSubsystem = new ClimberSubsystem(new ClimberIOTalonFX());
        intakeSubsystem = new IntakeSubsystem(new IntakeIOTalonFX());
        shooterSubsystem = new ShooterSubsystem(new ShooterIOTalonFX());
        hopperSubsystem = new HopperSubsystem(new HopperIOTalonFX());

        /* DATA FLOW:
        Vision IO (interface) connected VisionIOLimelight;
        ↓
        VisionSubsystem received data from VisionIOLimelight, then do calculations
        wrapped results into VisionFusionResults
        ↓
        Drive receive results, then add vision measurement

         */
        /*
        ANOTHER DATA FLOW:
        Drive <-> odometry pose history -> provided to vision to help calculation

         */

        visionIO = new VisionIOLimelight(() -> driveSubsystem.poseEstimator.getEstimatedPosition());
        visionSubsystem = new VisionSubsystem(visionIO, odometryHistory, driveSubsystem);
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

        climberIO = new ClimberIOSim();
        climberSubsystem = new ClimberSubsystem(climberIO);
        intakeSubsystem = new IntakeSubsystem(new IntakeIOSim());
        shooterSubsystem = new ShooterSubsystem(new ShooterIOTalonFX());
        hopperSubsystem = new HopperSubsystem(new HopperIOTalonFX());

        visionSubsystem =
            new VisionSubsystem(
                new VisionIOPhotonVisionSim(
                    () -> driveSimulation.getSimulatedDriveTrainPose(),
                    VisionConstant.robotToBackCam,
                    VisionConstant.robotToFrontCam),
                odometryHistory,
                driveSubsystem);

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
        visionSubsystem = new VisionSubsystem(visionIO, odometryHistory, driveSubsystem);

        break;
    }

    autoSubsystem =
        new AutoSubsystem(driveSubsystem, shooterSubsystem, climberSubsystem, hopperSubsystem);

    // Set up auto routines
    // autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());
    // building autochooser
    autoChooser = AutoBuilder.buildAutoChooser();

    // putting chooser on dashboard
    SmartDashboard.putData("Auto Chooser", autoChooser);

    //    // for registered commands
    autoChooser.setDefaultOption("StartBottomToTower", autoSubsystem.StartBottomToTower());
    autoChooser.addOption("bottomStartToShootOnly", autoSubsystem.bottomStartToShootOnly());
    autoChooser.addOption("topStartToShootOnly", autoSubsystem.topStartToShootOnly());
    autoChooser.addOption("midStartToShootOnly", autoSubsystem.midStartToShootOnly());
    autoChooser.addOption("StartTopToTower", autoSubsystem.StartTopToTower());
    autoChooser.addOption("StartMidToTower", autoSubsystem.StartMidToTower());
    autoChooser.addOption(
        "StartBottomShootIntakeEndL1", autoSubsystem.StartBottomShootIntakeEndL1());
    autoChooser.addOption("StartTopShootIntakeEndL1", autoSubsystem.StartTopShootIntakeEndL1());
    autoChooser.addOption("StartMidShootIntakeEndL1", autoSubsystem.StartMidShootIntakeEndL1());
    autoChooser.addOption("StartBottomShootEndL1", autoSubsystem.StartBottomShootEndL1());
    autoChooser.addOption("StartTopShootEndL1", autoSubsystem.StartTopShootEndL1());
    autoChooser.addOption("StartMidShootEndL1", autoSubsystem.StartMidShootEndL1());

    // Set up SysId routines
    autoChooser.addOption(
        "DriveSubsystem Wheel Radius Characterization",
        DriveCommands.wheelRadiusCharacterization(driveSubsystem));
    autoChooser.addOption(
        "DriveSubsystem Simple FF Characterization",
        DriveCommands.feedforwardCharacterization(driveSubsystem));
    autoChooser.addOption(
        "DriveSubsystem SysId (Quasistatic Forward)",
        driveSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "DriveSubsystem SysId (Quasistatic Reverse)",
        driveSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "DriveSubsystem SysId (Dynamic Forward)",
        driveSubsystem.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "DriveSubsystem SysId (Dynamic Reverse)",
        driveSubsystem.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link Joystick} or {@link
   * XboxController}), and then passing it to a {@link JoystickButton}.
   *
   * @return
   */
  private void configureButtonBindings() {
    // Default command, normal field-relative driveSubsystem
    driveSubsystem.setDefaultCommand(
        DriveCommands.joystickDrive(
            driveSubsystem,
            () -> -driverController.getLeftY(),
            () -> -driverController.getLeftX(),
            () -> -driverController.getRightX()));

    // Lock to 0° when A button is held
    driverController
        .a()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                driveSubsystem,
                () -> -driverController.getLeftY(),
                () -> -driverController.getLeftX(),
                () -> Rotation2d.kZero));

    // Switch to X pattern when X button is pressed
    //    driverController.x().onTrue(Commands.runOnce(driveSubsystem::stopWithX, driveSubsystem));
    driverController
        .x()
        .whileTrue(new AutoAimCommand(driveSubsystem, () -> driveSubsystem.getPose()));

    // Reset gyro / odometry
    final Runnable resetOdometry =
        Constants.currentMode == Constants.Mode.SIM
            ? () -> resetSimulation(new Pose2d(3, 3, new Rotation2d()))
            : () ->
                driveSubsystem.setPose(
                    new Pose2d(driveSubsystem.getPose().getTranslation(), new Rotation2d()));
    driverController.start().onTrue(Commands.runOnce(resetOdometry).ignoringDisable(true));
    // Left bumper Intake deployed and stowed
    // intake Commands
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

    // Hopper reverse while right bu
    // mper held
    operatorController
        .rightBumper()
        .whileTrue(
            Commands.startEnd(
                hopperSubsystem::runConveyorReverse,
                hopperSubsystem::stopConveyor,
                hopperSubsystem));

    // shooter Commands
    double distance = 1;
    operatorController
        .rightTrigger()
        .onTrue(ShooterCommands.storeDistance(shooterSubsystem, distance))
        .whileTrue(ShooterCommands.revFlywheels(shooterSubsystem, hopperSubsystem))
        .onFalse(ShooterCommands.reset(shooterSubsystem, hopperSubsystem));

    // Climber Setpoint Commands
    operatorController
        .a()
        // .and(operatorController.back())
        .onTrue(ClimberCommands.climbToGround(climberSubsystem));
    operatorController
        .x()
        // .and(operatorController.back())
        .onTrue(ClimberCommands.climbToLevelOne(climberSubsystem));
    operatorController
        .y()
        // .and(operatorController.back())
        .onTrue(ClimberCommands.climbToLevelTwo(climberSubsystem));
    operatorController
        .b()
        //  .and(operatorController.back())
        .onTrue(ClimberCommands.climbToLevelThree(climberSubsystem));

    // servo command
    operatorController
        .povUp()
        .and(operatorController.back())
        .onTrue(ClimberCommands.climberServoUp(climberSubsystem));
    operatorController
        .povDown()
        .and(operatorController.back())
        .onTrue(ClimberCommands.climberServoDown(climberSubsystem));
    new Trigger(
            () ->
                (operatorController.getRightY()) > 0.1 && operatorController.back().getAsBoolean())
        .whileTrue(ClimberCommands.baseServoUp(climberSubsystem));
    new Trigger(
            () ->
                (operatorController.getRightY()) < -0.1 && operatorController.back().getAsBoolean())
        .whileTrue(ClimberCommands.baseServoDown(climberSubsystem));

    // manual climb command
    new Trigger(
            () ->
                Math.abs(operatorController.getLeftY()) > 0.1
                    && operatorController.back().getAsBoolean())
        .whileTrue(ClimberCommands.joystickClimb(climberSubsystem, operatorController::getLeftY));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {

    return autoChooser.getSelected();
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
