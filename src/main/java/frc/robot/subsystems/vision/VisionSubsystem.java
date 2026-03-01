package frc.robot.subsystems.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.VisionConstant;
import frc.robot.subsystems.drive.Drive;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Consumer;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class VisionSubsystem extends SubsystemBase {
  /// ******************************
  /// so I'm a data collector
  /// ******************************

  private final VisionIO visionio;
  private final OdometryHistory odometryHistory;
  private final Supplier<Pose2d> pose2dSupplier;
  private final Drive drive;

  // need to be non static
  private double lastGyroTimestamp;
  private double lastGyroDegs;

  private final VisionIOInputsAutoLogged CamOneInputs = new VisionIOInputsAutoLogged();
  private final VisionIOInputsAutoLogged CamTwoInputs = new VisionIOInputsAutoLogged();

  private VisionMode pipelineState = null;
  private final SendableChooser<VisionMode> visionModeChooser = new SendableChooser<>();

  public VisionSubsystem(
      VisionIO visionio,
      Supplier<Pose2d> pose2dSupplier,
      OdometryHistory odometryHistory,
      Drive drive) {
    this.visionio = visionio;
    this.pose2dSupplier = pose2dSupplier;
    this.odometryHistory = odometryHistory;
    this.drive = drive;
    this.lastGyroTimestamp = Timer.getFPGATimestamp();
    this.lastGyroDegs = pose2dSupplier.get().getRotation().getDegrees();

    visionModeChooser.addOption("DISABLED", VisionMode.DISABLED);
    visionModeChooser.addOption("AUTONOMOUS", VisionMode.AUTONOMOUS);
    visionModeChooser.addOption("TELEOP", VisionMode.TELEOP);

    SmartDashboard.putData("Vision Mode Chooser", visionModeChooser);
  }

  @Override
  public void periodic() {
    visionio.updateInputs(CamOneInputs, CamTwoInputs);
    Logger.processInputs("Vision/side", CamOneInputs);
    logAutoAimInputs(pose2dSupplier);
    Logger.processInputs("Vision/front_shooter", CamTwoInputs);

    VisionMode manualSelectMode = visionModeChooser.getSelected();
    VisionMode finalMode = (manualSelectMode != decideVisionMode()) ? manualSelectMode : decideVisionMode();
    applyContVisionMode(finalMode);

    // they are separated because we don't want to feed pipeline continuously
    if (pipelineState == null || finalMode != pipelineState) {
      pipelineState = finalMode;
        applyOnceVisionMode(finalMode);
    }

    /// one
    var maybeMTA = processMegaTags(CamOneInputs);
    /// two
    var maybeMTB = processMegaTags(CamTwoInputs);

    /// if we have good estimation from MTA, then use MTA; the other way around for MTB as well
    /// if both does not provide good estimation, then we fuse both results
    // it will be packaged into VisionPoseFusion
    Optional<VisionFusionResults> acceptedInputs = Optional.empty();
    if (maybeMTA.isPresent() != maybeMTB.isPresent()) {
      acceptedInputs = maybeMTA.isPresent() ? maybeMTA : maybeMTB;
    } else if (maybeMTA.isPresent() && maybeMTB.isPresent()) {
      acceptedInputs = Optional.of(getFuseEstimation(maybeMTA.get(), maybeMTB.get()));
      Logger.recordOutput("Vision/UsedAllCamera", true);
    }

    acceptedInputs.ifPresent(
        visionFusionResults -> {
          drive.addVisionMeasurement(
              visionFusionResults.getVisionRobotPoseMeters(),
              visionFusionResults.getTimestampSeconds(),
              visionFusionResults.getVisionMeasurementStdDevs());
          Logger.recordOutput("Vision/SuccessfullyFused", true);
        });
  }
  /*
  ********************
  ********************
  SEPARATION LINE
  ********************
  ********************
  */

  private VisionFusionResults getFuseEstimation(VisionFusionResults a, VisionFusionResults b) {
    // Ensure b is the newer measurement
    if (b.getTimestampSeconds() < a.getTimestampSeconds()) {
      VisionFusionResults tmp = a;
      a = b;
      b = tmp;
    }

    // Preview both estimates to the same timestamp
    Transform2d a_T_b =
        odometryHistory
            .getPoseAt(b.getTimestampSeconds())
            .minus(odometryHistory.getPoseAt(a.getTimestampSeconds()));

    Pose2d poseA = a.getVisionRobotPoseMeters().transformBy(a_T_b);
    Pose2d poseB = b.getVisionRobotPoseMeters();

    // Inverse‑variance weighting
    var varianceA = a.getVisionMeasurementStdDevs().elementTimes(a.getVisionMeasurementStdDevs());
    var varianceB = b.getVisionMeasurementStdDevs().elementTimes(b.getVisionMeasurementStdDevs());

    Rotation2d fusedHeading = poseB.getRotation();
    if (varianceA.get(2, 0) < VisionConstant.kLargeVariance
        && varianceB.get(2, 0) < VisionConstant.kLargeVariance) {
      fusedHeading =
          new Rotation2d(
              poseA.getRotation().getCos() / varianceA.get(2, 0)
                  + poseB.getRotation().getCos() / varianceB.get(2, 0),
              poseA.getRotation().getSin() / varianceA.get(2, 0)
                  + poseB.getRotation().getSin() / varianceB.get(2, 0));
    }

    double weightAx = 1.0 / varianceA.get(0, 0);
    double weightAy = 1.0 / varianceA.get(1, 0);
    double weightBx = 1.0 / varianceB.get(0, 0);
    double weightBy = 1.0 / varianceB.get(1, 0);

    Pose2d fusedPose =
        new Pose2d(
            new Translation2d(
                (poseA.getTranslation().getX() * weightAx
                        + poseB.getTranslation().getX() * weightBx)
                    / (weightAx + weightBx),
                (poseA.getTranslation().getY() * weightAy
                        + poseB.getTranslation().getY() * weightBy)
                    / (weightAy + weightBy)),
            fusedHeading);

    Matrix<N3, N1> fusedStdDev =
        VecBuilder.fill(
            Math.sqrt(1.0 / (weightAx + weightBx)),
            Math.sqrt(1.0 / (weightAy + weightBy)),
            Math.sqrt(1.0 / (1.0 / varianceA.get(2, 0) + 1.0 / varianceB.get(2, 0))));

    int numTags = a.getNumTags() + b.getNumTags();
    double time = b.getTimestampSeconds();

    return new VisionFusionResults(fusedPose, time, fusedStdDev, numTags);
  }

  private Optional<VisionFusionResults> processMegaTags(VisionIOInputsAutoLogged inputs) {
    Optional<VisionFusionResults> estimateOrEmpty = Optional.empty();
    if (!isValidInputs(inputs)) {
      return Optional.empty();
    }
    Optional<VisionFusionResults> mtEstimate = processMTPoseEstimate(inputs);
    if (mtEstimate.isPresent()) {
      estimateOrEmpty = mtEstimate;
    }
    return estimateOrEmpty;
  }

  private Optional<VisionFusionResults> processMTPoseEstimate(VisionIOInputsAutoLogged inputs) {

    // Single‑tag extra checks
    if (inputs.tagCount < 2) {
      if (inputs.minAmbiguity > VisionConstant.kMinAmbiguityToFlip) {
        return Optional.empty();
      }
    }

    if (proportionalDistance(inputs) > VisionConstant.maxDistanceFromRobotToApril) {
      return Optional.empty();
    }

    if (inputs.ta < VisionConstant.kTagMinAreaForSingleTagMegatag) {
      return Optional.empty();
    }

    if (getGyroChange(pose2dSupplier) > VisionConstant.maxGyroChange) {
      return Optional.empty();
    }

    return Optional.of(
        new VisionFusionResults(
            inputs.megaTagPose,
            inputs.timestamp,
            VecBuilder.fill(
                inputs.rawStdDev[0], inputs.rawStdDev[1], VisionConstant.kLargeVariance),
            inputs.tagCount));
  }

  /// are we a valid pose?
  /// yes sir!
  private boolean isValidInputs(VisionIOInputsAutoLogged inputs) {
    if (inputs.tagCount <= 0) return false;
    if (!inputs.hasMegaTag2) return false;
    return inputs.megaTagPose != null;
  }

  private double proportionalDistance(VisionIOInputsAutoLogged inputs) {
    // if there is no target, or TA is too small (error)
    if (!inputs.hasTarget || inputs.ta < 0.0001) return 999;

    return 1.0 / Math.sqrt(inputs.ta);
  }

  private double getGyroChange(Supplier<Pose2d> supplier) {
    double currentTime = Timer.getFPGATimestamp();
    double currentGyroDegs = supplier.get().getRotation().getDegrees();
    double deltaTime = currentTime - lastGyroTimestamp;
    double deltaGryoDegs = currentGyroDegs - lastGyroDegs;

    lastGyroTimestamp = currentTime;
    lastGyroDegs = currentGyroDegs;

    // ignore this gyro change result
    if (deltaTime < 1e-6) return 0;

    return Math.abs(deltaGryoDegs / deltaTime);
  }

  private void logAutoAimInputs(Supplier<Pose2d> supplier) {
    double angle = AutoAimCalculation.getAngleToHub(supplier.get()).getDegrees();
    double[] targetHubPose2d =
        new double[] {
          AutoAimCalculation.getTargetHubPose2d().getX(),
          AutoAimCalculation.getTargetHubPose2d().getY(),
          AutoAimCalculation.getTargetHubPose2d().getRotation().getDegrees()
        };
    double distance = AutoAimCalculation.getDistanceFromRobotToHub(supplier.get());

    Logger.recordOutput("Vision/Aim/CurrentHubPose", targetHubPose2d);
    Logger.recordOutput("Vision/Aim/AngleToHub", angle);
    Logger.recordOutput("Vision/Aim/DistanceToHub", distance);
  }

  public void visualizeAprilTags(VisionIOInputsAutoLogged inputs, Supplier<Pose2d> robotPose) {
    Pose3d robot3d = new Pose3d(robotPose.get());
    List<Pose3d> linePoses = new ArrayList<>();
    for (var readAprilTagIDs : inputs.rawAprilTagID) {
      Optional<Pose3d> aprilTagPose =
          VisionConstant.aprilTagFieldLayout.getTagPose(readAprilTagIDs);
      if (aprilTagPose.isPresent()) {
        linePoses.add(aprilTagPose.get());
        linePoses.add(robot3d);
      }
    }
    Logger.recordOutput("seenAprilTags", linePoses.toArray(new Pose3d[0]));
  }

  private VisionMode decideVisionMode() {
    if (DriverStation.isDisabled()) {
      return VisionMode.DISABLED;
    } else if (DriverStation.isAutonomous()) {
      return VisionMode.AUTONOMOUS;
    } else if (DriverStation.isTeleop()) {
      return VisionMode.TELEOP;
    }
    return VisionMode.DISABLED;
  }

  private static final String[] CAMERAS = {VisionConstant.LIMELIGHT_A, VisionConstant.LIMELIGHT_B};

  private void forAllCameras(Consumer<String> action) {
    for (String cameraName : CAMERAS) {
      action.accept(cameraName);
    }
  }

  private void applyAllPipeline(int pipelineNumber) {
    forAllCameras(cam -> visionio.setPipeline(cam, pipelineNumber));
  }

  private void applyAllIMU(int mode) {
    forAllCameras(cam -> visionio.setIMUMode(cam, mode));
  }

  private void applyAllIMUAlphaAssist() {
    forAllCameras(
        cam -> visionio.setIMUAssistAlpha(cam, VisionConstant.complementaryFilterAlphaIMU));
  }

  private void applyAllOrientation(){
      forAllCameras(
              cam ->
                      visionio.setRobotOrientation(cam, pose2dSupplier.get().getRotation().getDegrees()));
  }

  private void applyOnceVisionMode(VisionMode visionMode) {

    switch (visionMode) {
      case DISABLED -> {
        applyAllPipeline(VisionConstant.PIPELINE_DEFAULT_OFF);
      }
      case AUTONOMOUS -> {
        applyAllPipeline(VisionConstant.PIPELINE_Autonomous);
      }

      case TELEOP -> {
        applyAllPipeline(VisionConstant.PIPELINE_Teleop);
      }
    }
  }

  private void applyContVisionMode(VisionMode visionMode){
      applyAllOrientation();
      Logger.recordOutput("Vision/CurrentVisionMode", visionMode.toString());
      switch (visionMode) {
          case DISABLED -> {
              applyAllIMU(1);
          }
          case AUTONOMOUS, TELEOP -> {
              applyAllIMU(4);
              applyAllIMUAlphaAssist();
          }

      }
  }
}

/// +++++++++++*****@@@@@@@%+++++++++++*##%*+++@@@@@@@@@@@@@#+++
/// ++++++++++*@@@@@@@@@@@@@*+++++**#%@@@@@@###@@@@@@@@@@@@@#+++
/// +++++++++++@@@@@@@@@@@@@@*#%@@@@@@@@@@@@@@@@@@@@@@@@@@@@*+++
/// +++++++++++%@@@@@@@@@@@%@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@%***+
/// ++++++++++++@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@%+
/// ++++++++++++*@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@*+
/// +++++++++#%@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@*+
/// ++++++++++@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@#*
/// +++++++++++*@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
/// +++++++++++%@@@@@@@@@@@@@@@@@@@@@@@@%%%%@@@@@@@@@@@@@@@@@@@@
/// ++++++++++#@@@@@@@@@@@@@@@@@@@@@@@@%%%@@@@@@@@@@@@@@@@@@@@@@
/// +++++++++#@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
/// ++++++++*@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
/// ++++++++#@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
/// +++++++*@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@#%@@@@@@@@@@@@@@
/// +++++++*@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@##@@@@@@@@@@@@@@
/// +++++++#@@@@@@@@@@@@@#%@@@@@@@@@@@@@@@@@@@@@#+++++@@@@@@@@@@
/// +++++++%@@@@@@@@@@@@@#@#++++++@@@@@@@@@@@%*+++#%#*++@@@@@@@@
/// ++++++*@@@@@@@@@@@@@%@+++#%#*++*##********++*@@@@@@%*@@%@@@@
/// ++++++*@@@@@@@@@@@@@@#+*@@@@@@@++*********++*@@@@@@@%*%#@@@@
/// ++++++*@@@@@@@@@@@@@@*+@@@@@@@@#+**********+*@@@@@@@%*++%@@@
/// ++++++%@@@@@@@@@@@@@@%*@@@@@@@@#+***********+*@@@@@@#++*%@@@
/// +++++#@@@@@@@@@@@@@@@#+*@@@@@@#++*************+*###*++++%@@@
/// ++++*@@@@@@@@@@@@@@@@*+++****+++*****************####***@@@#
/// +++*@@@@@@@@@@@@@@@@@@*++++++++**************************#*#
/// ++*#@@@@@@@@@@@@##@@@@*****#****************************@@@@
/// +++++++++*@@@@@@@@##%@@*****************##*************@@@@%
/// ++++++++*#%@%#*+#%@@@@@#***************%##%**********%@@@@@#
/// ++++++++++++++++++#@@@@@@#*************@###%******#@@@@@@@#*
/// +++++++++++++++++++#@@@@@@@@#**********#%##%***#@@@@@@@@@@@+
/// ++++++++++++++++++++#@@@@@@@@@@@@%##****%%@@@@@@@%#@@@@@@@@@
/// ++++++++++++++++++*@@@@@@@@@@@*#%@@@@@@@@@@@@@%%@@@@@@@@@@@@
/// +++++++++++++++++%@@@@@@@@@@@@@@@@@@@###%%%##%@@@@@@@@@@@@@@
