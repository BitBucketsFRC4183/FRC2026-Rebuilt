package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import java.util.function.Supplier;

import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

import static edu.wpi.first.cameraserver.CameraServer.addCamera;

public class VisionIOPhotonVisionSim implements VisionIO {
  // set the stage // this is basically allinone
  private VisionIO visionio;
  private VisionIOInputs frontCamInputs = new VisionIOInputs();
  private VisionIOInputs backCamInputs = new VisionIOInputs();
  private VisionSystemSim visionSim;
  private PhotonCameraSim frontCameraSim;
  private PhotonCameraSim backCameraSim;

  private final Supplier<Pose2d> poseSupplier;

  private final Transform3d robotToFront;
  private final Transform3d robotToBack;

  public VisionIOPhotonVisionSim(String cameraName, Transform3d robotToCamera) {

    if (visionSim == null) {
      visionSim = new VisionSystemSim("PhotonSim");
      visionSim.addAprilTags(VisionConstant.aprilTagFieldLayout);
    }
    frontCameraSim = new PhotonCameraSim(frontCameraSim, robotToFront);
  }


  public void periodic(){
    visionio.updateInputs(frontCamInputs, backCamInputs);
  }

  @Override
  public void updateInputs(VisionIOInputs frontCamInputs, VisionIOInputs backCamInputs) {
    frontCamInputs.cameraConnected = ;
    //    visionSim.update(poseSupplier.get());
//    super.updateInputs(frontCamInputs, backCamInputs);
  }
}
