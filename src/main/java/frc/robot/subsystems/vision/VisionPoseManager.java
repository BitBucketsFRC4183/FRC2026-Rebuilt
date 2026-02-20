package frc.robot.subsystems.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.constants.VisionConstant;
import frc.robot.subsystems.drive.DriveSubsystem;
import org.littletonrobotics.junction.AutoLogOutput;

import java.util.Map;

public class VisionPoseManager {
/// gime the best pose in your shop

    public void processAllCameraData(Map<String, VisionIOInputsAutoLogged> inputMap){
        VisionIOInputsAutoLogged bestPose = getBestPoseAtCurrentTime(inputMap);

        if (bestPose == null)
            return;
        Matrix<N3, N1> StdDev = computeVisionStdDev(bestPose.tagCount);
    }

    /// complicated concept
    /// standard error is inversely proportional to standard deviation / sqrt(TagCount)
    // suggested link: https://docs.ufpr.br/~danielsantos/ProbabilisticRobotics.pdf
    @AutoLogOutput(key="Vision/PoseTuning/VisionStdDev")
    private Matrix<N3, N1> computeVisionStdDev(int tagCount){

        /// simple detection system, will change later
        // already tuned measurement, just tune as much as possible
        if (tagCount <=0) tagCount = 1;

        if (tagCount <= 1){
            return VisionConstant.GlobalVisionMeasurementStdDevs;
        }else if (tagCount == 2){
            return VisionConstant.TwoTag_VisionMeasurementStdDevs;
        }else if (tagCount ==3){
            return VisionConstant.ThreeTag_VisionMeasurementStdDevs;
        }

        double standardError = 1.0 / Math.sqrt(tagCount);
        Matrix<N3, N1> stdDev = new Matrix<>(N3.instance, N1.instance);
        stdDev.set(0, 0, VisionConstant.baseX* standardError); // x
        stdDev.set(1, 0, VisionConstant.baseY* standardError); // y
        stdDev.set(2, 0, VisionConstant.baseTheta* standardError); // θ
        return stdDev;
    }

    /// are we a valid pose?
    /// yes sir!
    private boolean isValidPose(VisionIOInputsAutoLogged inputs) {
        if (inputs.tagCount <= 0) return false;
        if (!inputs.hasMegaTag2) return false;
        if (inputs.estimatedRobotPose == null) return false;

        return true;
    }

    /// Area image is inversely proportional to 1 / distance^2
    /// distance estimated => Sqrt(1 / TA) = 1/ sqrt(TA)
    // Concept really hard, check these links
    // https://astronomy.stackexchange.com/questions/39686/intuition-about-why-gravity-is-inversely-proportional-to-exactly-square-of-dista
    // https://theswissbay.ch/pdf/Gentoomen%20Library/Artificial%20Intelligence/Computer%20Vision/Computer%20Vision%20A%20Modern%20Approach%20-%20Forsyth%20%2C%20Ponce.pdf#:~:text=In%20this%20case%2C%20the%20patch%20is%20small%2C,dA%20cos%CE%B8%20r2%20where%20the%20terminology%20is
    private double proportionalDistance(VisionIOInputsAutoLogged inputs){
        //if there is no target, or TA is too small (error)
        if (!inputs.hasTarget || inputs.ta < 0.0001)
            return 999;

        return 1.0 / Math.sqrt(inputs.ta);
    }

    //see best pose
    private VisionIOInputsAutoLogged getBestPoseAtCurrentTime(Map<String, VisionIOInputsAutoLogged> inputMap){
        VisionIOInputsAutoLogged bestInputs = null;
        double maxDistance = VisionConstant.maxDistanceFromRobotToApril;
        for (VisionIOInputsAutoLogged inputs : inputMap.values()){
            if (!isValidPose(inputs))
                continue;
            double proportionalDistance = proportionalDistance(inputs);
            if (proportionalDistance <maxDistance){
                maxDistance = proportionalDistance;
                bestInputs = inputs;
            }
        }
        return bestInputs;
}

@AutoLogOutput(key="Vision/PoseTuning/BestPose")
    private Pose2d getBestPose(Map<String, VisionIOInputsAutoLogged> inputMap){
        return getBestPoseAtCurrentTime(inputMap).megaTagPose;
}

}
