package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.constants.VisionConstant;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;
import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;

public class VisionIOPhotonVisionSim extends VisionIOLimelight {
  // set the stage
  // this is basically allinone yooo
  private VisionSystemSim visionSim;

  private final PhotonCamera PHOTON_FRONT;
  private final PhotonCamera PHOTON_BACK;

  private final PhotonCameraSim frontCamSim;
  private final PhotonCameraSim backCamSim;

  private final Supplier<Pose2d> poseSupplier;

  public VisionIOPhotonVisionSim(Supplier<Pose2d> poseSupplier) // the parameter
      {
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
    //    frontCamSim.enableRawStream(true);
    //    frontCamSim.enableProcessedStream(true);
    //    frontCamSim.enableDrawWireframe(true);
    //
    //    backCamSim.enableRawStream(true);
    //    backCamSim.enableProcessedStream(true);
    //    backCamSim.enableDrawWireframe(true);

    // add properties that's it
    visionSim.addCamera(frontCamSim, VisionConstant.robotToFrontCam);
    visionSim.addCamera(backCamSim, VisionConstant.robotToBackCam);
  }

  @Override
  public void updateInputs(VisionIOInputs frontCamInputs, VisionIOInputs backCamInputs) {

    visionSim.update(poseSupplier.get());
    NetworkTable table = NetworkTableInstance.getDefault().getTable(VisionConstant.LIMELIGHT_A);
    NetworkTable tableB = NetworkTableInstance.getDefault().getTable(VisionConstant.LIMELIGHT_B);

    /// write to the table we have for limelight
    writeToTable(PHOTON_FRONT.getAllUnreadResults(), table, frontCamSim);
    writeToTable(PHOTON_BACK.getAllUnreadResults(), tableB, backCamSim);
    super.updateInputs(frontCamInputs, backCamInputs);
  }

  private List<Double> getBotpose(
      Transform3d fieldToCamera,
      int numTags,
      PhotonPipelineResult result,
      PhotonCameraSim cameraSim) {
    if (result == null || result.targets.isEmpty()) return null;

    Optional<Transform3d> optRobotToCamera =
        visionSim.getRobotToCamera(cameraSim, Timer.getFPGATimestamp());
    Pose3d fieldToRobot;
    if (optRobotToCamera.isPresent()) {
      Transform3d cameraToRobot = optRobotToCamera.get().inverse();
      Pose3d robotPose3d =
          new Pose3d(fieldToCamera.getTranslation(), fieldToCamera.getRotation())
              .transformBy(cameraToRobot);
      fieldToRobot = robotPose3d;
    } else {
      fieldToRobot = new Pose3d(fieldToCamera.getTranslation(), fieldToCamera.getRotation());
    }

    List<Double> pose_data =
        new ArrayList<>(
            Arrays.asList(
                fieldToRobot.getX(),
                fieldToRobot.getY(),
                fieldToRobot.getZ(),
                0.0,
                0.0,
                fieldToRobot.getRotation().getMeasureZ().in(Units.Degree),
                result.metadata.getLatencyMillis(),
                (double) numTags,
                0.0,
                0.0,
                result.getBestTarget().getArea()));

    for (var target : result.targets) {
      pose_data.addAll(
          Arrays.asList(
              (double) target.getFiducialId(),
              target.getYaw(), // txnc
              target.getPitch(), // tync
              target.area, // ta
              0.0, // distToCamera
              0.0, // distToRobot
              target.getPoseAmbiguity() // ambiguity
              ));
    }
    return pose_data;
  }

  private void writeToTable(
      List<PhotonPipelineResult> results, NetworkTable table, PhotonCameraSim cameraSim) {
    boolean seesTarget = false;

    for (var result : results) {
      List<Double> pose_data = null;
      if (result.getMultiTagResult().isPresent()) {
        var multiTagResult = result.getMultiTagResult().get();
        Transform3d best = multiTagResult.estimatedPose.best;
        pose_data = getBotpose(best, multiTagResult.fiducialIDsUsed.size(), result, cameraSim);
      } else if (result.hasTargets()) {
        var bestTarget = result.getBestTarget();
        Transform3d best =
            VisionConstant.aprilTagFieldLayout
                .getTagPose(bestTarget.getFiducialId())
                .get()
                .minus(Pose3d.kZero)
                .plus(bestTarget.bestCameraToTarget.inverse());

        pose_data = getBotpose(best, 1, result, cameraSim);
      }

      if (pose_data != null) {
        table
            .getEntry("botpose_wpiblue")
            .setDoubleArray(pose_data.stream().mapToDouble(Double::doubleValue).toArray());
        table
            .getEntry("botpose_orb_wpiblue")
            .setDoubleArray(pose_data.stream().mapToDouble(Double::doubleValue).toArray());
        // [MT1x, MT1y, MT1z, MT1roll, MT1pitch, MT1Yaw, MT2x, MT2y, MT2z, MT2roll,
        // MT2pitch,
        // MT2yaw]
        table
            .getEntry("stddevs")
            .setDoubleArray(
                new Double[] {0.3, 0.3, 0.0, 0.0, 0.0, 0.3, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
        seesTarget = true;
      }
      table.getEntry("cl").setDouble(result.metadata.getLatencyMillis());
    }
    table.getEntry("tv").setInteger(seesTarget ? 1 : 0);
  }
}
