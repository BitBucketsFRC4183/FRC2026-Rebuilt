package frc.robot.subsystems.vision;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.DriveSubsystem;
import org.littletonrobotics.junction.Logger;

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

  private final VisionIO cameraFront;
  private final VisionIO cameraSide;
  private final DriveSubsystem drive;

  // loggable data
  private final VisionIOInputsAutoLogged cameraFrontInputs = new VisionIOInputsAutoLogged();
  private final VisionIOInputsAutoLogged cameraSideInputs = new VisionIOInputsAutoLogged();

  public VisionSubsystem(VisionIO cameraFront, VisionIO cameraSide, DriveSubsystem driveSubsystem) {
    this.cameraFront = cameraFront;
    this.cameraSide = cameraSide;
    this.drive = driveSubsystem;
  }

  @Override
  public void periodic() {
    cameraFront.updateInputs(cameraFrontInputs);
    cameraSide.updateInputs(cameraSideInputs);
    // Logger.processInputs("Vision/front", cameraInputsAll.get(VisionConstant.LIMELIGHT_FRONT));
    Logger.processInputs("Vision/cameraFront", cameraFrontInputs);
    Logger.processInputs("Vision/cameraSide", cameraSideInputs);

    // only use cameraFront for now
    if (cameraFrontInputs.hasMegaTag2) {
      drive.addVisionMeasurement(
          cameraFrontInputs.megaTagPose,
          cameraFrontInputs.timestamp,
          VecBuilder.fill(.7, .7, 9999999));
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
