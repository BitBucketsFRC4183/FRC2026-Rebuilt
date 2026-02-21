package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import java.util.Map;
import java.util.TreeMap;

public class OdometryHistory {
  TreeMap<Double, Pose2d> history = new TreeMap<>();
  // 20ms -> 1 second = 200 entries
  private final int maxHistorySize = 300;

  public void addPose(double timestamps, Pose2d pose) {
    history.put(timestamps, pose);

    while (history.size() > maxHistorySize) {
      history.pollFirstEntry();
    }
  }

  public Pose2d getPoseAt(double timestamp) {
    if (history.isEmpty()) {
      return new Pose2d();
    }
    Map.Entry<Double, Pose2d> entry = history.floorEntry(timestamp);
    if (entry != null) return entry.getValue();
    return history.firstEntry().getValue();
  }
}
