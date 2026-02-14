package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.AimConstants;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.vision.AimController;
import frc.robot.subsystems.vision.HopperTracker;

import java.util.function.Supplier;

/// I just found it is basically same thing in one of the drive command
/// but that's fine
public class AutoAimCommand extends Command {
    private final DriveSubsystem driveSubsystem;
    private final Supplier<Pose2d> poseSupplier;
    private final AimController aimController;
    public AutoAimCommand(DriveSubsystem driveSubsystem, Supplier<Pose2d> poseSupplier){
        this.driveSubsystem = driveSubsystem;
        this.poseSupplier = poseSupplier;
        this.aimController = new AimController(
                AimConstants.KP,
                AimConstants.KI,
                AimConstants.KD
        );
        addRequirements(driveSubsystem);
    }

  @Override
  public void initialize(){
        aimController.reset();
        //initialize smartdashboard
      SmartDashboard.putNumber("Aim/KP", AimConstants.KP);
      SmartDashboard.putNumber("Aim/KI", AimConstants.KI);
      SmartDashboard.putNumber("Aim/KD", AimConstants.KD);
}
  public void execute() {
        //so get values and store
        double kp = SmartDashboard.getNumber("Aim/KP", AimConstants.KP);
      double ki = SmartDashboard.getNumber("Aim/KI", AimConstants.KI);
      double kd = SmartDashboard.getNumber("Aim/KD", AimConstants.KD);

      //refer the PID from aimController
      //so if you chance any of the smartdashboard value, it will dynamically become PID tuning values
      //no need to build
      aimController.getAimPID().setP(kp);
      aimController.getAimPID().setI(ki);
      aimController.getAimPID().setD(kd);

      Pose2d robotPose = poseSupplier.get();
      double currentRad = robotPose.getRotation().getRadians();
      double targetRad = HopperTracker.getAngleToHubRad(robotPose);
      double omega = aimController.calculateFromAngles(currentRad, targetRad);

      omega = aimController.shapeOutput(omega, driveSubsystem.getMaxAngularSpeedRadPerSec());

      driveSubsystem.runOmega(omega);
    }

  @Override
    public  boolean isFinished(){
        return aimController.atTarget();
  }
}
