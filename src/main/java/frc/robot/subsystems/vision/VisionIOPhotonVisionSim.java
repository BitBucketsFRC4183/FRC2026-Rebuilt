package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.constants.VisionConstant;
import java.util.List;
import java.util.function.Supplier;
import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;

public class VisionIOPhotonVisionSim implements VisionIO {
  // set the stage
  // this is basically allinone yooo
  private VisionSystemSim visionSim;

  private PhotonCamera PHOTON_FRONT;
  private PhotonCamera PHOTON_BACK;

  private PhotonCameraSim frontCamSim;
  private PhotonCameraSim backCamSim;
  private final Supplier<Pose2d> poseSupplier;

  public VisionIOPhotonVisionSim(
      Supplier<Pose2d> poseSupplier,
      Transform3d robotToFrontCam,
      Transform3d robotToBackCam) // the parameter
      {
    // now create the stage
    this.poseSupplier = poseSupplier;

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
    camProps.setAvgLatencyMs(20);
    camProps.setCalibError(0.35, 0.5);
    camProps.setLatencyStdDevMs(5);
    camProps.setExposureTimeMs(0.65);

    // lets put "real" camera to the sim construction
    frontCamSim = new PhotonCameraSim(PHOTON_FRONT, camProps);
    frontCamSim.setMinTargetAreaPixels(1000);
    backCamSim = new PhotonCameraSim(PHOTON_BACK, camProps);
    backCamSim.setMinTargetAreaPixels(1000);

    // streaming
    frontCamSim.enableRawStream(true);
    frontCamSim.enableProcessedStream(true);
    frontCamSim.enableDrawWireframe(true);

    backCamSim.enableRawStream(true);
    backCamSim.enableProcessedStream(true);
    backCamSim.enableDrawWireframe(true);

    // add properties that's it
    visionSim.addCamera(frontCamSim, robotToFrontCam);
    visionSim.addCamera(backCamSim, robotToBackCam);
  }

  @Override
  public void updateInputs(VisionIOInputs frontCamInputs, VisionIOInputs backCamInputs) {
    visionSim.update(poseSupplier.get());
    List<PhotonPipelineResult> visionResult = PHOTON_FRONT.getAllUnreadResults();

    
    frontCamInputs.cameraConnected = PHOTON_FRONT.isConnected();
    frontCamInputs.aprilTagIDNumber = 0;
    frontCamInputs.timestamp = visionResult.lastIndexOf(frontCamInputs);
    
  }


}
