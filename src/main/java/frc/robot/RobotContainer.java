// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
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
import frc.robot.commands.AutoAimCommand;
import frc.robot.commands.ClimberCommands;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.IntakeCommands;
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

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final DriveSubsystem driveSubsystem;
  private final HopperSubsystem hopperSubsystem;
  private final ShooterSubsystem shooterSubsystem;
  private final IntakeSubsystem intakeSubsystem;
  private final ShooterSim shooterSim;
  private VisionSubsystem visionSubsystem;
  private AutoSubsystem autoSubsystem;
  private final SendableChooser<Command> autoChooser;

  // Added missing subsystem fields
  private ClimberIO climberIO;
  private ClimberSubsystem climberSubsystem;
  private VisionIO visionIO;

  // Toggle state for left bumper

  // Controller
  private final CommandXboxController driverController = new CommandXboxController(0);
  private final CommandXboxController operatorController = new CommandXboxController(1);

  // Dashboard inputs

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        // ModuleIOTalonFX is intended for modules with TalonFX driveSubsystem, TalonFX turn, and
        // a CANcoder
        driveSubsystem =
            new DriveSubsystem(
                new GyroIOPigeon2(),
                new ModuleIOTalonFXAnalog(TunerConstants.FrontLeft),
                new ModuleIOTalonFXAnalog(TunerConstants.FrontRight),
                new ModuleIOTalonFXAnalog(TunerConstants.BackLeft),
                new ModuleIOTalonFXAnalog(TunerConstants.BackRight));

        climberIO = new ClimberIOTalonFX();
        climberSubsystem = new ClimberSubsystem(climberIO);

        intakeSubsystem = new IntakeSubsystem(new IntakeIOTalonFX());
        shooterSubsystem = new ShooterSubsystem(new ShooterIOTalonFX());
        hopperSubsystem = new HopperSubsystem(new HopperIOTalonFX());

        // register named commands

        // register named commands

        //        NamedCommands.registerCommand("StartBottomToTower",
        // autoSubystem.StartBottomToTower());
        //        NamedCommands.registerCommand("bottomStartToShootOnly",
        // autoSubystem.bottomStartToShootOnly());
        //        NamedCommands.registerCommand("topStartToShootOnly",
        // autoSubystem.topStartToShootOnly());
        //        NamedCommands.registerCommand("midStartToShootOnly", autoSubystem.
        // midStartToShootOnly());
        //        NamedCommands.registerCommand("StartTopToTower", autoSubystem.StartTopToTower());
        //        NamedCommands.registerCommand("StartMidToTower", autoSubystem.StartMidToTower());
        //        NamedCommands.registerCommand("StartBottomShootIntakeEndL1",
        // autoSubystem.StartBottomShootIntakeEndL1());
        //        NamedCommands.registerCommand("StartTopShootIntakeEndL1",
        // autoSubystem.StartTopShootIntakeEndL1());
        //        NamedCommands.registerCommand("StartMidShootIntakeEndL1",
        // autoSubystem.StartMidShootIntakeEndL1());
        //        NamedCommands.registerCommand("StartTopShootEndL1",
        // autoSubystem.StartTopShootEndL1());
        //        NamedCommands.registerCommand("StartBottomShootEndL1",
        // autoSubystem.StartBottomShootEndL1());
        //        NamedCommands.registerCommand("StartMidShootEndL1",
        // autoSubystem.StartMidShootEndL1());

        visionIO = new VisionIOLimelight(() -> driveSubsystem.poseEstimator.getEstimatedPosition());
        visionSubsystem = new VisionSubsystem(visionIO, driveSubsystem);

        shooterSim = new ShooterSim();
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        driveSubsystem =
            new DriveSubsystem(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight));

        climberIO = new ClimberIOSim();
        climberSubsystem = new ClimberSubsystem(climberIO);

        intakeSubsystem = new IntakeSubsystem(new IntakeIOSim());
        shooterSubsystem = new ShooterSubsystem(new ShooterIOTalonFX());
        hopperSubsystem = new HopperSubsystem(new HopperIOTalonFX());

        visionSubsystem =
            new VisionSubsystem(
                new VisionIOPhotonVisionSim(
                    driveSubsystem.poseSupplierForSim,
                    VisionConstant.robotToBackCam,
                    VisionConstant.robotToFrontCam),
                driveSubsystem);

        shooterSim = new ShooterSim();
        break;
        // thinking to what

      default:
        // Replayed robot, disable IO implementations
        driveSubsystem =
            new DriveSubsystem(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});

        climberIO = new ClimberIOSim();
        climberSubsystem = new ClimberSubsystem(climberIO);

        intakeSubsystem = new IntakeSubsystem(new IntakeIOSim());
        shooterSubsystem = new ShooterSubsystem(new ShooterIOTalonFX());
        hopperSubsystem = new HopperSubsystem(new HopperIOTalonFX());

        visionSubsystem = null;

        shooterSim = new ShooterSim();
        break;
    }
    autoSubsystem =
        new AutoSubsystem(driveSubsystem, shooterSubsystem, climberSubsystem, hopperSubsystem);

    NamedCommands.registerCommand("StartBottomToTower", autoSubsystem.StartBottomToTower());
    NamedCommands.registerCommand("bottomStartToShootOnly", autoSubsystem.bottomStartToShootOnly());
    NamedCommands.registerCommand("topStartToShootOnly", autoSubsystem.topStartToShootOnly());
    NamedCommands.registerCommand("midStartToShootOnly", autoSubsystem.midStartToShootOnly());
    NamedCommands.registerCommand("StartTopToTower", autoSubsystem.StartTopToTower());
    NamedCommands.registerCommand("StartMidToTower", autoSubsystem.StartMidToTower());
    NamedCommands.registerCommand(
        "StartBottomShootIntakeEndL1", autoSubsystem.StartBottomShootIntakeEndL1());
    NamedCommands.registerCommand(
        "StartTopShootIntakeEndL1", autoSubsystem.StartTopShootIntakeEndL1());
    NamedCommands.registerCommand(
        "StartMidShootIntakeEndL1", autoSubsystem.StartMidShootIntakeEndL1());
    NamedCommands.registerCommand("StartTopShootEndL1", autoSubsystem.StartTopShootEndL1());
    NamedCommands.registerCommand("StartBottomShootEndL1", autoSubsystem.StartBottomShootEndL1());
    NamedCommands.registerCommand("StartMidShootEndL1", autoSubsystem.StartMidShootEndL1());
    // Set up auto routines
    // autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());
    // building autochooser
    autoChooser = AutoBuilder.buildAutoChooser();

    // putting chooser on dashboard
    SmartDashboard.putData("Auto Chooser", autoChooser);

    // for registered commands
    autoChooser.addOption("StartBottomToTower", autoSubsystem.StartBottomToTower());
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
    operatorController
        .leftTrigger()
        .whileTrue(
            IntakeCommands.intake(intakeSubsystem).onlyIf(() -> intakeSubsystem.isExtended()));

    // Hopper reverse while right bumper held
    operatorController
        .rightBumper()
        .whileTrue(
            Commands.startEnd(
                hopperSubsystem::runConveyorReverse,
                hopperSubsystem::stopConveyor,
                hopperSubsystem));

    // Intake Control Motors

    // servo command
    operatorController.povUp().onTrue(ClimberCommands.climberServoUp(climberSubsystem));
    operatorController.povDown().onTrue(ClimberCommands.climberServoDown(climberSubsystem));
    new Trigger(
            () ->
                (operatorController.getRightY()) > 0.1 && operatorController.back().getAsBoolean())
        .whileTrue(ClimberCommands.baseServoUp(climberSubsystem));
    new Trigger(
            () ->
                (operatorController.getRightY()) < 0.1 && operatorController.back().getAsBoolean())
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
}
