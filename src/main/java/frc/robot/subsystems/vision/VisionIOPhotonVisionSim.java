package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.constants.VisionConstant;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import java.util.List;
import java.util.function.Supplier;
import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

public class VisionIOPhotonVisionSim implements VisionIO {
  // set the stage
  // this is basically allinone yooo
  private VisionSystemSim visionSim;

  private PhotonCamera PHOTON_FRONT;
  private PhotonCamera PHOTON_BACK;

  private PhotonCameraSim frontCamSim;
  private PhotonCameraSim backCamSim;
  private final Supplier<Pose2d> poseSupplier;

  DriveSubsystem driveSubsystem =
      new DriveSubsystem(
          new GyroIO() {},
          new ModuleIOSim(TunerConstants.FrontLeft),
          new ModuleIOSim(TunerConstants.FrontRight),
          new ModuleIOSim(TunerConstants.BackLeft),
          new ModuleIOSim(TunerConstants.BackRight));

  public VisionIOPhotonVisionSim(
      Supplier<Pose2d> poseSupplier,
      Transform3d robotToFrontCam,
      Transform3d robotToBackCam,
      DriveSubsystem driveSubsystem) // the parameter
      {
    // now create the stage
    this.poseSupplier = poseSupplier;
    //    poseSupplier = driveSubsystem.poseSupplierForSim;

    if (visionSim == null) {
      visionSim = new VisionSystemSim("PhotonSim");
      visionSim.addAprilTags(VisionConstant.aprilTagFieldLayout);
    }
    // so we will have some Photon camera
    PHOTON_FRONT = new PhotonCamera("PhotonFront");
    PHOTON_BACK = new PhotonCamera("PhotonBack");

    // sim props
    SimCameraProperties camProps = new SimCameraProperties();
    camProps.setFPS(30);
    camProps.setAvgLatencyMs(35);
    camProps.setCalibError(0.25, 0.08);

    // lets put "real" camera to the sim construction
    frontCamSim = new PhotonCameraSim(PHOTON_FRONT, camProps);
    backCamSim = new PhotonCameraSim(PHOTON_BACK, camProps);

    // streaming
    frontCamSim.enableRawStream(true);
    frontCamSim.enableProcessedStream(true);
    frontCamSim.enableDrawWireframe(true);

    backCamSim.enableRawStream(true);
    backCamSim.enableProcessedStream(true);
    backCamSim.enableDrawWireframe(true);

    // add properties that's it
    // yooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooo
    visionSim.addCamera(frontCamSim, robotToFrontCam);
    visionSim.addCamera(backCamSim, robotToBackCam);
  }

  @Override
  public void updateInputs(VisionIOInputs frontCamInputs, VisionIOInputs backCamInputs) {
    visionSim.update(poseSupplier.get());
    List visionResult = PHOTON_FRONT.getAllUnreadResults();

    frontCamInputs.estimatedRobotPose = visionSim.getRobotPose().toPose2d();
    frontCamInputs.cameraConnected = PHOTON_FRONT.isConnected();
    frontCamInputs.fiducialID = PHOTON_FRONT.getPipelineIndex();
    frontCamInputs.timestamp = visionResult.lastIndexOf(frontCamInputs);
  }
}
