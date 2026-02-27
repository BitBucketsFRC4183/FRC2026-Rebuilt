package frc.robot.subsystems.vision;

import frc.robot.constants.VisionConstant;
import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

public class VisionIOPhotonVisionSim extends VisionIOLimelight {
  // set the stage
  // this is basically allinone yooo
  private VisionSystemSim visionSim;

  private final PhotonCamera PHOTON_FRONT;
  private final PhotonCamera PHOTON_BACK;

  private final PhotonCameraSim frontCamSim;
  private final PhotonCameraSim backCamSim;

  public VisionIOPhotonVisionSim() // the parameter
      {

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
    visionSim.addCamera(frontCamSim, VisionConstant.robotToFrontCam);
    visionSim.addCamera(backCamSim, VisionConstant.robotToBackCam);
  }

  @Override
  public void updateInputs(VisionIOInputs frontCamInputs, VisionIOInputs backCamInputs) {
    super.updateInputs(frontCamInputs, backCamInputs);
  }
}
