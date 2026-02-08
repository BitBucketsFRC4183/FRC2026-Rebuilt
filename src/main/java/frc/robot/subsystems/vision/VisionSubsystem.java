package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.DriveSubsystem;
import org.littletonrobotics.junction.Logger;

///so shooter game 2026: concept
//there is a april tag right below the hopper.
//for example, red alliance is apriltag 10
//we want to bind to these apriltags when trigger: aim only at these apriltags,
//-> robot pose will always face the hopper form rotational movement, like an arc robot move back and forth
//-> and shoot balls driver can still move around, then we based on the position, form another arc, and etc
//-> shooter already calculated angular velocity to best shoot
//-> we want to still aim the specific april tag, even though camera can't see it.

/// how to implement
//april tag adjust robot pose, we will need a field pose to aim the hopper

/// identify roles of different subsystem
//

public class VisionSubsystem extends SubsystemBase {
  private final VisionIO visionio;
  private final VisionIOInputsAutoLogged frontCamInputs = new VisionIOInputsAutoLogged();
  private final VisionIOInputsAutoLogged backCamInputs = new VisionIOInputsAutoLogged();
  // loggable data
  private final DriveSubsystem driveSubsystem;

  public VisionSubsystem(VisionIO io, DriveSubsystem driveSubsystem) {
    this.visionio = io;
    this.driveSubsystem = driveSubsystem;
  }

  @Override
  public void periodic() {
    visionio.updateInputs(frontCamInputs, backCamInputs);

    //    Logger.recordOutput("Area of Taget", inputs.ta);
    // how big AprilTag is in the camera frame
    // basically, 3%-> far;
    // 80%-> takes big portion of the frame, AprilTag is near

    // fusion; add vision measurement
    Pose2d visionFusedPose = null;
    double visionFusedTimestamps = 0.0;

    if (frontCamInputs.hasTarget && backCamInputs.hasTarget) {
      visionFusedPose = averagePose(frontCamInputs.megaTagPose, backCamInputs.megaTagPose);
      visionFusedTimestamps = Math.max(frontCamInputs.timestamp, backCamInputs.timestamp);

    } else if (frontCamInputs.hasTarget) {
      visionFusedPose = frontCamInputs.megaTagPose;
      visionFusedTimestamps = frontCamInputs.timestamp;

    } else if (backCamInputs.hasTarget) {
      visionFusedPose = backCamInputs.megaTagPose;
      visionFusedTimestamps = backCamInputs.timestamp;
    }
//add vision measurement
    if (visionFusedPose != null) {
      driveSubsystem.addVisionMeasurement(visionFusedPose, visionFusedTimestamps);
    }
//log loggable inputs
    // Stringkey: the path, distinguish where the data wants to go to; custom naming
    Logger.processInputs("Vision/front", frontCamInputs);
    Logger.processInputs("Vision/back", backCamInputs);

    //filter the best tag!
  }

  private Pose2d averagePose(Pose2d a, Pose2d b) {
    double avgX = (a.getX() + b.getX()) / 2.0;
    double avgY = (a.getY() + b.getY()) / 2.0;

    var rotationA = a.getRotation();
    var rotationB = b.getRotation();
    var avgRotation = a.getRotation().interpolate(b.getRotation(), 0.5);

    return new Pose2d(avgX, avgY, avgRotation);
  }
}