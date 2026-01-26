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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.constants.ForearmConstants;
import frc.robot.constants.VisionConstant;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.climber.ClimberIOTalonFX;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.forearm.ForearmIOSparkMax;
import frc.robot.subsystems.forearm.ForearmSubsystem;
import frc.robot.subsystems.hopper.HopperIOSparkMax;
import frc.robot.subsystems.hopper.HopperSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.vision.*;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final DriveSubsystem driveSubsystem;
  // private final AutoSubsystem autoSubsystem;
  private final HopperSubsystem hopperSubsystem;
  private final ForearmSubsystem forearmSubsystem;
  private final ShooterSubsystem shooterSubsystem;
  private VisionSubsystem visionSubsystem;
  private VisionIOLimelight visionIO;
  private ClimberSubsystem climberSubsystem;

  // Toggle state for left bumper
  private boolean forearmExtended = false;

  // Controller
  private final CommandXboxController driver = new CommandXboxController(0);
  private final CommandXboxController operator = new CommandXboxController(1);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

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

        // The ModuleIOTalonFXS implementation provides an example implementation for
        // TalonFXS controller connected to a CANdi with a PWM encoder. The
        // implementations
        // of ModuleIOTalonFX, ModuleIOTalonFXS, and ModuleIOSpark (from the Spark
        // swerve
        // template) can be freely intermixed to support alternative hardware
        // arrangements.
        // Please see the AdvantageKit template documentation for more information:
        // https://docs.advantagekit.org/getting-started/template-projects/talonfx-swerve-template#custom-module-implementations
        //
        // drive =
        // new Drive(
        // new GyroIOPigeon2(),
        // new ModuleIOTalonFXS(TunerConstants.FrontLeft),
        // new ModuleIOTalonFXS(TunerConstants.FrontRight),
        // new ModuleIOTalonFXS(TunerConstants.BackLeft),
        // new ModuleIOTalonFXS(TunerConstants.BackRight));
        visionIO = new VisionIOLimelight(() -> driveSubsystem.poseEstimator.getEstimatedPosition());
        visionSubsystem =
            new VisionSubsystem(new VisionIOInputsAutoLogged(), visionIO, driveSubsystem);
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
        visionSubsystem =
            new VisionSubsystem(
                new VisionIOInputsAutoLogged(),
                new VisionIOPhotonVisionSim(
                    driveSubsystem.poseSupplierForSim,
                    VisionConstant.robotToBackCam,
                    VisionConstant.robotToFrontCam,
                    driveSubsystem),
                driveSubsystem);
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
        break;
    }

    // Set up auto routines
    this.hopperSubsystem = new HopperSubsystem(new HopperIOSparkMax());
    this.forearmSubsystem = new ForearmSubsystem(new ForearmIOSparkMax());
    this.shooterSubsystem = new ShooterSubsystem();
    this.climberSubsystem = new ClimberSubsystem(new ClimberIOTalonFX());
    // this.autoSubsystem = new AutoSubsystem(DriveSubsystem driveSubsystem, ClimbSubsystem climber,
    // ShooterSubystem shooter);

    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

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
   */
  private void configureButtonBindings() {
    // Default command, normal field-relative driveSubsystem
    driveSubsystem.setDefaultCommand(
        DriveCommands.joystickDrive(
            driveSubsystem,
            () -> -driver.getLeftY(),
            () -> -driver.getLeftX(),
            () -> -driver.getRightX()));

    // Lock to 0° when A button is held
    driver
        .a()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                driveSubsystem,
                () -> -driver.getLeftY(),
                () -> -driver.getLeftX(),
                () -> Rotation2d.kZero));

    // Switch to X pattern when X button is pressed
    driver.x().onTrue(Commands.runOnce(driveSubsystem::stopWithX, driveSubsystem));

    // Reset gyro to 0° when B button is pressed
    driver
        .b()
        .onTrue(
            Commands.runOnce(
                    () ->
                        driveSubsystem.setPose(
                            new Pose2d(
                                driveSubsystem.getPose().getTranslation(), Rotation2d.kZero)),
                    driveSubsystem)
                .ignoringDisable(true));

    driver
        // Left Bumper Triggers Intake extended mode and Intake retract mode
        .leftBumper()
        .onTrue(
            Commands.runOnce(
                () -> {
                  if (forearmExtended) {
                    forearmSubsystem.runForearmManual(ForearmConstants.MANUAL_RETRACT_PERCENT);
                  } else {
                    forearmSubsystem.runForearmManual(ForearmConstants.MANUAL_EXTEND_PERCENT);
                  }
                  forearmExtended = !forearmExtended;
                },
                forearmSubsystem));

    // Left trigger: run intake while held
    new Trigger(() -> driver.getLeftTriggerAxis() > 0.1)
        .whileTrue(
            Commands.run(
                () -> forearmSubsystem.runIntake(ForearmConstants.INTAKE_IN_PERCENT),
                forearmSubsystem))
        .onFalse(Commands.runOnce(forearmSubsystem::stopIntake, forearmSubsystem));

    operator.a().onTrue(Commands.runOnce(() -> climberSubsystem.moveClimbToGround()));
    operator.b().onTrue(Commands.runOnce(() -> climberSubsystem.moveClimbToLevel1()));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
