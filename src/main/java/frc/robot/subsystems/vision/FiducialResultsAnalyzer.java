package frc.robot.subsystems.vision;

import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructSerializable;
import frc.robot.LimelightHelpers;

import java.nio.ByteBuffer;
import java.util.Arrays;
import java.util.Objects;

//public record FiducialResultsAnalyzer
//        (int id, double ambiguity, double area)
//        implements StructSerializable {
//
//
//    public static FiducialResultsAnalyzer fromLimelight
//            (LimelightHelpers.RawFiducial fiducial) {
//        if (fiducial == null) {
//            return null;
//        }
//        return new FiducialResultsAnalyzer(
//                fiducial.id, fiducial.ambiguity, fiducial.ta);
//    }
//
//    public static FiducialResultsAnalyzer[] fromLimelight(LimelightHelpers.RawFiducial[] fiducials) {
//        if (fiducials == null) {
//            return new FiducialResultsAnalyzer[0];
//        }
//        return Arrays.stream(fiducials)
//                .map(FiducialResultsAnalyzer::fromLimelight)
//                .filter(Objects::nonNull)
//                .toArray(FiducialResultsAnalyzer[]::new);
//    }
//}