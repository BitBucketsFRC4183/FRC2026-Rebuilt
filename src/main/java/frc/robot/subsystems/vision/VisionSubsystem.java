package frc.robot.subsystems.vision;

import static frc.robot.subsystems.vision.VisionMode.AUTONOMOUS;
import static frc.robot.subsystems.vision.VisionMode.TELEOP;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.constants.VisionConstants;
import frc.robot.subsystems.drive.Drive;
import java.util.Optional;
import java.util.function.Consumer;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

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

  private VisionMode currentVisionMode;
  private VisionMode lastVisionMode;

  private boolean oneCameraMode = true;

  private final LoggedDashboardChooser<VisionMode> visionModeChooser;

  public VisionSubsystem(VisionIO visionio, Supplier<Pose2d> pose2dSupplier, Drive drive) {
    this.visionio = visionio;
    this.pose2dSupplier = pose2dSupplier;
    this.drive = drive;
    this.lastGyroTimestamp = Timer.getFPGATimestamp();
    this.lastGyroDegs = pose2dSupplier.get().getRotation().getDegrees();

    visionModeChooser =
        new LoggedDashboardChooser<VisionMode>("/SmartDashboard/VisionMode Chooser");
    visionModeChooser.addDefaultOption("DISABLED", VisionMode.DISABLED);
    visionModeChooser.addOption("AUTONOMOUS", AUTONOMOUS);
    visionModeChooser.addOption("TELEOP", TELEOP);
    visionModeChooser.onChange(
        (mode) -> {
          currentVisionMode = mode;
        });
    SmartDashboard.putData("Vision Mode Chooser", visionModeChooser.getSendableChooser());
  }

  // seed once when reseting the pose of the robot
  public void setLimelightIMUGyro(Rotation2d rotation) {
    LimelightHelpers.SetIMUMode(VisionConstants.LIMELIGHT_A, 1);

    double heading = rotation.getDegrees();
    forAllCameras(cam -> visionio.setRobotOrientation(cam, heading));
    Logger.recordOutput("Vision/GyroInputs/WhenSetPose:robotHeading", heading);
    Command delaySwitch =
        Commands.sequence(
            Commands.waitSeconds(0.1),
            Commands.runOnce(() -> LimelightHelpers.SetIMUMode(VisionConstants.LIMELIGHT_A, 3)));
    delaySwitch.schedule();
  }

  @Override
  public void periodic() {
    if (DriverStation.isDisabled()) {
      currentVisionMode = VisionMode.DISABLED;
    } else if (DriverStation.isTeleopEnabled()) {
      currentVisionMode = TELEOP;
    } else if (DriverStation.isAutonomousEnabled()) {
      currentVisionMode = AUTONOMOUS;
    }

    visionio.updateInputs(CamOneInputs, CamTwoInputs);
    Logger.processInputs("Vision/front", CamOneInputs);
    Logger.processInputs("Vision/side", CamTwoInputs);

    logAutoAimInputs(pose2dSupplier);
    Logger.recordOutput("Vision/Mode/currentGyroVisionMode", currentVisionMode.toString());

    setVisionPipelineForAllCameras(TELEOP);
    seedGyroVisionMode(currentVisionMode);

    /// one
    var maybeMTA = processMegaTags(CamOneInputs, "front");
    /// two
    var maybeMTB = processMegaTags(CamTwoInputs, "side");

    /// if we have good estimation from MTA, then use MTA; the other way around for MTB as well
    /// if both does not provide good estimation, then we fuse both results
    // it will be packaged into VisionPoseFusion
    Optional<VisionFusionResults> acceptedInputs = Optional.empty();
    if (maybeMTA.isPresent() != maybeMTB.isPresent()) {
      acceptedInputs = maybeMTA.isPresent() ? maybeMTA : maybeMTB;
    } else if (maybeMTA.isPresent() && maybeMTB.isPresent()) {
      acceptedInputs = Optional.of(getFuseEstimation(maybeMTA.get(), maybeMTB.get()));
    }

    Logger.recordOutput("Vision/HasVisionEstimate", acceptedInputs.isPresent());
    acceptedInputs.ifPresent(
        visionFusionResults -> {
          drive.addVisionMeasurement(
              visionFusionResults.getVisionRobotPoseMeters(),
              visionFusionResults.getTimestampSeconds(),
              visionFusionResults.getVisionMeasurementStdDevs());
          Logger.recordOutput(
              "Vision/Estimator/robotPose", visionFusionResults.getVisionRobotPoseMeters());
          Logger.recordOutput(
              "Vision/Estimator/timestamps", visionFusionResults.getTimestampSeconds());
          Logger.recordOutput(
              "Vision/Estimator/stdDev", visionFusionResults.getVisionMeasurementStdDevs());
        });
  }

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
    if (varianceA.get(2, 0) < VisionConstants.kLargeVariance
        && varianceB.get(2, 0) < VisionConstants.kLargeVariance) {
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

  private Optional<VisionFusionResults> processMegaTags(
      VisionIOInputsAutoLogged inputs, String cameraName) {
    boolean basicFilterPass = passedBasicFilter(inputs, cameraName);
    boolean advancedFilterPass = passedAdvancedFilter(inputs, cameraName);
    boolean bothFilterPass;

    if (oneCameraMode) {
      bothFilterPass = basicFilterPass;
    } else {
      bothFilterPass = basicFilterPass && advancedFilterPass;
    }

    /// we expect these to be green
    Logger.recordOutput("Vision/" + cameraName + "/BasicPass", basicFilterPass);
    Logger.recordOutput("Vision/" + cameraName + "/AdvancedPass", advancedFilterPass);
    Logger.recordOutput("Vision/" + cameraName + "/ValidResults", bothFilterPass);

    if (!bothFilterPass) {
      return Optional.empty();
    }

    double xStd = 0.3;
    double yStd = 0.3;
    //    double theta = inputs.rawStdDev[11];

    return Optional.of(
        new VisionFusionResults(
            inputs.megaTagPose,
            inputs.timestamp,
            VecBuilder.fill(xStd, yStd, VisionConstants.kLargeVariance),
            //            VecBuilder.fill(xStd, yStd, theta),
            inputs.tagCount));
  }

  private boolean passedAdvancedFilter(VisionIOInputsAutoLogged inputs, String cameraName) {
    double distance = proportionalDistance(inputs);
    boolean distanceValid = distance < VisionConstants.maxDistanceFromRobotToApril;
    boolean areaValid = inputs.ta > VisionConstants.kTagMinAreaForSingleTagMegatag;

    Logger.recordOutput("Vision/" + cameraName + "/Filter/ProportionalDistance", distance);
    Logger.recordOutput("Vision/" + cameraName + "/Filter/DistanceValid", distanceValid);
    Logger.recordOutput("Vision/" + cameraName + "/Filter/AreaValid", areaValid);
    return distanceValid && areaValid;
  }

  private boolean passedBasicFilter(VisionIOInputsAutoLogged inputs, String cameraName) {
    boolean tagValid = isValidInputs(inputs);
    Logger.recordOutput("Vision/" + cameraName + "/Filter/TagValid", tagValid);
    //    if (!isValidInputs(inputs)) {
    //      boolean TagCountValid = false;
    //      return false;
    //    }
    double gyroChange = getGyroChange(pose2dSupplier);
    Logger.recordOutput("Vision/" + cameraName + "/Filter/GyroChange", gyroChange);

    boolean gyroValid = gyroChange <= VisionConstants.maxGyroChange;
    Logger.recordOutput("Vision/" + cameraName + "/Filter/GyroValid", gyroValid);

    //    if (getGyroChange(pose2dSupplier) > VisionConstants.maxGyroChange) {
    //      return false;
    //    }
    return tagValid && gyroValid;
  }

  private void logAutoAimInputs(Supplier<Pose2d> supplier) {
    Logger.recordOutput("Aim/CurrentHubPose", AutoAimUtil.getTargetHubPose3d());
    Logger.recordOutput("Aim/TargetAngle", AutoAimUtil.getAngleToHub(supplier));
    Logger.recordOutput("Aim/DistanceToHub", AutoAimUtil.getDistanceToHub(supplier));
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

    double d = Math.abs(deltaGryoDegs / deltaTime);

    //    Logger.recordOutput("Vision/GyroChangeUsed", d);
    //    Logger.recordOutput("Vision/GyroDegsUsed", currentGyroDegs);
    return d;
  }

  /// /////
  /// all helper
  private static final String[] CAMERAS = {
    VisionConstants.LIMELIGHT_A, VisionConstants.LIMELIGHT_B
  };

  private void forAllCameras(Consumer<String> action) {
    for (String cameraName : CAMERAS) {
      action.accept(cameraName);
    }
  }

  private void setPipelineForAllCameras(int pipelineNumber) {
    forAllCameras(cam -> visionio.setPipeline(cam, pipelineNumber));
  }

  /// set orientation from pose supplier
  private void setIMUModeForAllCameras(int mode) {
    LimelightHelpers.SetIMUMode(VisionConstants.LIMELIGHT_A, mode);
  }

  private void applyIMUAssistForAllCameras() {
    forAllCameras(
        cam -> visionio.setIMUAssistAlpha(cam, VisionConstants.complementaryFilterAlphaIMU));
  }

  private void setIMUOrientationForAllCameras() {
    double robotHeading = pose2dSupplier.get().getRotation().getDegrees();
    forAllCameras(cam -> visionio.setRobotOrientation(cam, robotHeading));
    Logger.recordOutput("Vision/GyroInputs/WhileDisabled:robotHeading", robotHeading);
  }

  /// all helper
  /// /////

  private void setVisionPipelineForAllCameras(VisionMode visionMode) {
    switch (visionMode) {
      case DISABLED -> {
        setPipelineForAllCameras(VisionConstants.PIPELINE_DEFAULT_OFF);
      }
      case AUTONOMOUS -> {
        setPipelineForAllCameras(VisionConstants.PIPELINE_Autonomous);
      }
      case TELEOP -> {
        setPipelineForAllCameras(VisionConstants.PIPELINE_Teleop);
      }
    }
    Logger.recordOutput("Vision/Mode/pipelineVisionMode", visionMode.toString());
  }

  private void seedGyroVisionMode(VisionMode visionMode) {
    switch (visionMode) {
      case DISABLED -> {
        LimelightHelpers.SetRobotOrientation_NoFlush(
            VisionConstants.LIMELIGHT_A, drive.getPose().getRotation().getDegrees(), 0, 0, 0, 0, 0);
        LimelightHelpers.SetIMUMode(VisionConstants.LIMELIGHT_A, 1);
      }
      case TELEOP, AUTONOMOUS -> LimelightHelpers.SetIMUMode(VisionConstants.LIMELIGHT_A, 3);
    }
  }

  //  @AutoLogOutput(key = "Aim/DistanceToHub")
  public double getHubDistanceMeter(Supplier<Pose2d> robotPose) {
    return AutoAimUtil.getDistanceToHub(robotPose);
  }

  //  private void logCameraData(String cameraName, VisionIOInputsAutoLogged inputs) {}
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
