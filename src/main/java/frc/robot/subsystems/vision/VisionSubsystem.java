package frc.robot.subsystems.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.constants.VisionConstant;
import org.littletonrobotics.junction.Logger;

import java.util.Optional;

/// shooter game 2026: concept & mechanics
// april tags, hub poses --> given
// need: accurate robot pose on the field
// just like gps, align robot pose to hub pose

/// how to implement

/// debug...
// reflection issues, possibly not this season --> soln: reduce exposue, reduce brightness
// calibration

public class VisionSubsystem extends SubsystemBase {
  /// ******************************
  /// so I'm a data collector
  /// ******************************

  private final VisionIO visionio;
  private final OdometryHistory odometryHistory;
  private final VisionIOInputsAutoLogged CamOneInputs = new VisionIOInputsAutoLogged();
  private final VisionIOInputsAutoLogged CamTwoInputs = new VisionIOInputsAutoLogged();

  public VisionSubsystem(VisionIO io, OdometryHistory odometryHistory) {
    this.visionio = io;
    this.odometryHistory = odometryHistory;

//    LimelightHelpers.SetIMUMode();
//    LimelightHelpers.setRewindEnabled("", true);
  }

  public void seedInternalIMU(){
    LimelightHelpers.SetIMUMode(VisionConstant.LIMELIGHT_FRONT, 1);
  }

  @Override
  public void periodic() {
    visionio.updateInputs(CamOneInputs, CamTwoInputs);

    /// one
    var maybeMTA = processMegaTags(CamOneInputs);
    /// two
    var maybeMTB = processMegaTags(CamTwoInputs);

    /// if we have good estimation from MTA, then use MTA; the other way around for MTB as well
    /// if both does not provide good estimation, then we fuse both results
   // it will be packaged into VisionPoseFusion, and
    Optional<VisionPoseFusion> acceptedInputs = Optional.empty();
    if (maybeMTA.isPresent() != maybeMTB.isPresent()) {
      acceptedInputs = maybeMTA.isPresent() ? maybeMTA : maybeMTB;
    } else if (maybeMTA.isPresent() && maybeMTB.isPresent()) {
      acceptedInputs = Optional.of(getFuseEstimation(maybeMTA.get(), maybeMTB.get()));
    }

    Logger.processInputs("Vision/front", CamOneInputs);
    Logger.processInputs("Vision/front_shooter", CamTwoInputs);
    }


    private VisionPoseFusion getFuseEstimation (VisionPoseFusion a, VisionPoseFusion b) {
      // Ensure b is the newer measurement
      if (b.getTimestampSeconds() < a.getTimestampSeconds()) {
        VisionPoseFusion tmp = a;
        a = b;
        b = tmp;
      }

      // Preview both estimates to the same timestamp
      Transform2d a_T_b =
              odometryHistory.getPoseAt(b.getTimestampSeconds())
                      .minus(odometryHistory.getPoseAt(a.getTimestampSeconds()));

      Pose2d poseA = a.getVisionRobotPoseMeters().transformBy(a_T_b);
      Pose2d poseB = b.getVisionRobotPoseMeters();

      // Inverse‑variance weighting
      var varianceA =
              a.getVisionMeasurementStdDevs().elementTimes(a.getVisionMeasurementStdDevs());
      var varianceB =
              b.getVisionMeasurementStdDevs().elementTimes(b.getVisionMeasurementStdDevs());

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

      return new VisionPoseFusion(fusedPose, time, fusedStdDev, numTags);
    }

    /// ********************
    /// ********************
    /// ********************
    /// ********************
    /// ********************

  public Optional<VisionPoseFusion> processMegaTags(VisionIOInputsAutoLogged inputs){
    Optional<VisionPoseFusion> estimateOrEmpty = Optional.empty();
    if (!isValidInputs(inputs)) {
      return Optional.empty();
    }
    Optional<VisionPoseFusion> mtEstimate = processMTPoseEstimate(inputs);
    if (mtEstimate.isPresent()){
      estimateOrEmpty = mtEstimate;
    }
    return estimateOrEmpty;
  }

  private Optional<VisionPoseFusion> processMTPoseEstimate(VisionIOInputsAutoLogged inputs) {

    // Single‑tag extra checks
    if (inputs.tagCount < 2) {
        if (inputs.minAmbiguity > VisionConstant.kMinAmbiguityToFlip){
          return Optional.empty();
        }
    }

    if (proportionalDistance(inputs)<VisionConstant.maxDistanceFromRobotToApril){
      return Optional.empty();
    }

    if (inputs.ta < VisionConstant.kTagMinAreaForSingleTagMegatag) {
      return Optional.empty();
    }

    return Optional.of(new VisionPoseFusion(
      inputs.megaTagPose,
      inputs.timestamp,
            VecBuilder.fill(inputs.rawStdDev[0], inputs.rawStdDev[1], inputs.rawStdDev[2]),
      inputs.tagCount));
  }

  /// are we a valid pose?
  /// yes sir!
  private boolean isValidInputs(VisionIOInputsAutoLogged inputs) {
    if (inputs.tagCount <= 0) return false;
    if (!inputs.hasMegaTag2) return false;
    if (inputs.estimatedRobotPose == null) return false;
    return true;
  }

  private double proportionalDistance(VisionIOInputsAutoLogged inputs){
    //if there is no target, or TA is too small (error)
    if (!inputs.hasTarget || inputs.ta < 0.0001)
      return 999;

    return 1.0 / Math.sqrt(inputs.ta);
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
