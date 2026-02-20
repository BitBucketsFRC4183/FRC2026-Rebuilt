package frc.robot.subsystems.vision;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.constants.AimConstants;
import org.littletonrobotics.junction.AutoLogOutput;

public class AimController {
  private final PIDController aimPID;
  private double lastError = 0;
  private double lastOmega = 0;

  public AimController(double kp, double ki, double kd) {
    aimPID = new PIDController(kp, ki, kd);
    aimPID.setTolerance(AimConstants.TOLERANCE_DEG);
  }

  /// calculate omega velocity (measured, target)
  public double calculateFromAngles(double currentRad, double targetRad) {
    lastError = MathUtil.angleModulus(targetRad - currentRad);
    lastOmega = aimPID.calculate(currentRad, targetRad);
    return lastOmega;
  }

  // enter drive's maxSpeed setting
  public double shapeOutput(double omega, double maxOmega) {
    if (Math.abs(omega) < AimConstants.OUTPUT_DEADBAND) return 0.0;
    // Math.copySign(magnitude, signSource)
    if (Math.abs(omega) < AimConstants.MINIMUM_OUTPUT) {
      omega = Math.copySign(AimConstants.MINIMUM_OUTPUT, omega);
    }

    // limited velocity
    return MathUtil.clamp(omega, -maxOmega, maxOmega);
  }

  // TODO
  public double calculateFromTx(double txDegrees) {
    double txRad = Math.toRadians(txDegrees);
    return aimPID.calculate(txRad, 0.0);
  }

  public PIDController getAimPID() {
    return aimPID;
  }

  @AutoLogOutput(key = "Vision/Aim/Omega")
  private double getOmegaLog() {
    return lastOmega;
  }

  @AutoLogOutput(key = "Vision/Aim/Error")
  private double getErrorLogDeg() {
    return Math.toDegrees(lastError);
  }

  @AutoLogOutput(key = "Vision/Aim/atTarget")
  public boolean atTarget() {
    return aimPID.atSetpoint();
  }

  public void reset() {
    aimPID.reset();
  }
}
