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
import java.util.Optional;
import java.util.function.Consumer;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class VisionSubsystem extends SubsystemBase {
  /// ******************************
  /// so I'm a data collector
  /// ******************************

  private final VisionIO visionio;
  private final Supplier<Pose2d> pose2dSupplier;
  private final Drive drive;

  // need to be non static
  private double lastGyroTimestamp;
  private double lastGyroDegs;

  private final VisionIOInputsAutoLogged CamOneInputs = new VisionIOInputsAutoLogged();
  private final VisionIOInputsAutoLogged CamTwoInputs = new VisionIOInputsAutoLogged();

  private VisionMode pipelineState = null;
  private VisionMode lastAutoMode = null;
  private VisionMode manualMode = null;

  private boolean oneCameraMode = true;

  private VisionMode defaultMode = VisionMode.DISABLED;

  private final SendableChooser<VisionMode> visionModeChooser = new SendableChooser<>();

  public VisionSubsystem(VisionIO visionio, Supplier<Pose2d> pose2dSupplier, Drive drive) {
    this.visionio = visionio;
    this.pose2dSupplier = pose2dSupplier;
    this.drive = drive;
    this.lastGyroTimestamp = Timer.getFPGATimestamp();
    this.lastGyroDegs = pose2dSupplier.get().getRotation().getDegrees();

    visionModeChooser.addOption("DISABLED", VisionMode.DISABLED);
    visionModeChooser.addOption("AUTONOMOUS", VisionMode.AUTONOMOUS);
    visionModeChooser.addOption("TELEOP", VisionMode.TELEOP);
    visionModeChooser.onChange(
        mode -> {
          manualMode = mode;
        });

    SmartDashboard.putData("Vision Mode Chooser", visionModeChooser);
  }

  @Override
  public void periodic() {
    visionio.updateInputs(CamOneInputs, CamTwoInputs);
    Logger.processInputs("Vision/side", CamOneInputs);
    Logger.processInputs("Vision/front_shooter", CamTwoInputs);

    /// logging
    logAutoAimInputs(pose2dSupplier);
    logAprilTagPose(CamOneInputs, pose2dSupplier);
    logAprilTagPose(CamTwoInputs, pose2dSupplier);

    if (getFinalVisionMode() != defaultMode) {
      defaultMode = getFinalVisionMode();
    }
    applyContVisionMode(defaultMode);

    // they are separated because we don't want to feed pipeline continuously
    if (defaultMode != pipelineState) {
      pipelineState = defaultMode;
      applyOnceVisionMode(defaultMode);
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
      Logger.recordOutput("Vision/UsedAllCamera", false);
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
        drive
            .getOdometryHistory()
            .getPoseAt(b.getTimestampSeconds())
            .minus(drive.getOdometryHistory().getPoseAt(a.getTimestampSeconds()));

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
    boolean isBasicFiltered = passedBasicFilter(inputs);
    boolean isAdvancedFiltered = passedAdvancedFilter(inputs);
    boolean useMeasurement;

    if (oneCameraMode) {
      useMeasurement = isBasicFiltered;

    } else {
      useMeasurement = isBasicFiltered && isAdvancedFiltered;
    }

    if (!useMeasurement) {
      Logger.recordOutput("Vision/FilterOutResults", true);
      return Optional.empty();
    }

    double xStd = inputs.rawStdDev[6];
    double yStd = inputs.rawStdDev[7];
    //    double theta = inputs.rawStdDev[11];

    Logger.recordOutput("Vision/FilterOutResults", !useMeasurement);

    return Optional.of(
        new VisionFusionResults(
            inputs.megaTagPose,
            inputs.timestamp,
            VecBuilder.fill(xStd, yStd, VisionConstant.kLargeVariance),
            //            VecBuilder.fill(xStd, yStd, theta),
            inputs.tagCount));
  }

  private boolean passedAdvancedFilter(VisionIOInputsAutoLogged inputs) {

    // Single‑tag extra checks
    //    if (inputs.tagCount < 2) {
    //      if (inputs.minAmbiguity > VisionConstant.kMinAmbiguityToFlip) {
    //        return Optional.empty();
    //      }
    //    }

    if (AutoAimCalculation.getDistanceFromRobotToHub(pose2dSupplier.get())
        > VisionConstant.maxDistanceFromRobotToApril) {
      return false;
    }
    if (inputs.ta < VisionConstant.kTagMinAreaForSingleTagMegatag) {
      return false;
    }
    return true;
  }

  private boolean passedBasicFilter(VisionIOInputsAutoLogged inputs) {

    if (!isValidInputs(inputs)) {
      return false;
    }

    if (getGyroChange(pose2dSupplier) > VisionConstant.maxGyroChange) {
      return false;
    }
    return true;
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
    double angle = AutoAimCalculation.getTargetAngle(supplier.get()).getDegrees();

    double distance = AutoAimCalculation.getDistanceFromRobotToHub(supplier.get());

    Logger.recordOutput("Vision/Aim/CurrentHubPose", AutoAimCalculation.getTargetHubPose3d());
    Logger.recordOutput("Vision/Aim/TargetAngle", angle);
    Logger.recordOutput("Vision/Aim/DistanceToHub", distance);
  }

  private void logAprilTagPose(VisionIOInputsAutoLogged inputs, Supplier<Pose2d> robotPose) {

    if (inputs.rawAprilTagID.length == 0) {
      return;
    } else {
      int[] ids = inputs.rawAprilTagID;
      Pose3d[] aprilTagPoses = new Pose3d[ids.length];
      for (int i = 0; i < ids.length; i++) {
        aprilTagPoses[i] = VisionConstant.aprilTagFieldLayout.getTagPose(ids[i]).get();
      }
      Logger.recordOutput("Vision/AprilTagPoses", aprilTagPoses);
    }
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

  private VisionMode getFinalVisionMode() {
    VisionMode autoMode = decideVisionMode();

    if (lastAutoMode == null) {
      lastAutoMode = autoMode;
    } else if (autoMode != lastAutoMode) {
      lastAutoMode = autoMode;
      manualMode = null;
    }

    if (manualMode != null) {
      return manualMode;
    } else {
      return autoMode;
    }
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

  private void applyAllOrientation() {
    forAllCameras(
        cam -> visionio.setRobotOrientation(cam, pose2dSupplier.get().getRotation().getDegrees()));
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

  private void applyContVisionMode(VisionMode visionMode) {
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
