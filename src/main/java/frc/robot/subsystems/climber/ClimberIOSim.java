package frc.robot.subsystems.climber;

import com.ctre.phoenix6.hardware.core.CoreTalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.constants.ClimberConstants;

public class ClimberIOSim implements ClimberIO {
  CoreTalonFX coreTalonFX = new CoreTalonFX(1);
  public final TalonFXSimState climberSimMotor = coreTalonFX.getSimState();

  private double simHeightMeters = 0.0;
  private double simVelocity = 0.0;
  private static final double maxHeight = ClimberConstants.rung1Position;
  double appliedVoltage = 0.0;
  private double targetHeightMeters = 0.0;
  private final SlewRateLimiter voltageLimiter = new SlewRateLimiter(12.0);

  private final Mechanism2d climberCanvas = new Mechanism2d(3, 3);
  private final MechanismRoot2d climberRoot = climberCanvas.getRoot("pivot", 1.5, 1.5);
  MechanismLigament2d climberModel =
          climberRoot.append(new MechanismLigament2d("climberMotor", 1, 0));

  public ClimberIOSim() {
    SmartDashboard.putData("Climber Data", climberCanvas);
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    double dt = 0.02;
    double maxSpeedMetersPerSec = 1.5; // tune this

    simVelocity = (appliedVoltage / 12.0) * maxSpeedMetersPerSec;

    simHeightMeters += simVelocity * dt;

    simHeightMeters = MathUtil.clamp(simHeightMeters, 0, maxHeight);

    if ((simHeightMeters == 0 && appliedVoltage < 0)
            || (simHeightMeters == maxHeight && appliedVoltage > 0)) {
      simVelocity = 0;
    }

    climberModel.setLength(simHeightMeters + 0.2);
    inputs.climberHeight = simHeightMeters;
    inputs.currentVoltage = appliedVoltage;

    climberSimMotor.setSupplyVoltage(appliedVoltage);

    //    simHeightMeters += simVelocity * dt;
    //    simHeightMeters = Math.max(0, Math.min(MAX_HEIGHT, simHeightMeters));
    //
    //    climberModel.setLength(simHeightMeters + 0.2);
    //
    //    inputs.climberHeight = simHeightMeters;
    //    double error = targetHeightMeters - simHeightMeters;
    //    double commandedVoltage = MathUtil.clamp(error * 6.0, -12.0, 12.0);
    //    appliedVoltage = voltageLimiter.calculate(commandedVoltage);
    //    inputs.currentVoltage = appliedVoltage;
    //    climberSimMotor.setSupplyVoltage(inputs.currentVoltage);
  }

  @Override
  public void setTargetHeight(double height) {
    targetHeightMeters = height;
    double error = height - simHeightMeters;
  }

  @Override
  public void stopClimb() {}

  @Override
  public double getCurrentHeight() {
    return simHeightMeters;
  }

  @Override
  public double getCurrentVoltage() {
    return appliedVoltage;
  }

  @Override
  public void setVoltage(double voltageSupplied) {
    if (voltageSupplied > 12.0) {
      appliedVoltage = 12.0;
    } else if (voltageSupplied < -12.0) {
      appliedVoltage = -12.0;
    } else {
      appliedVoltage = voltageSupplied;
    }
  }
}
