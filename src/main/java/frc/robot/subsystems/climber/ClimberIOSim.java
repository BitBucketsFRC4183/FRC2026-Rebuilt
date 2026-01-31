package frc.robot.subsystems.climber;

import com.ctre.phoenix6.hardware.core.CoreTalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.constants.ClimberConstants;

public class ClimberIOSim implements ClimberIO {
  CoreTalonFX coreTalonFX = new CoreTalonFX(1);
  public final TalonFXSimState climberSimMotor = coreTalonFX.getSimState();

  private final Mechanism2d climerCanvas = new Mechanism2d(3, 3);
  private final MechanismRoot2d climerRoot = climerCanvas.getRoot("pivot", 1.5, 1.5);
  MechanismLigament2d climberModel =
      climerRoot.append(new MechanismLigament2d("climberMotor", 1, 0));

  public ClimberIOSim() {
    SmartDashboard.putData("Climber Data", climerCanvas);
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    climberSimMotor.setRawRotorPosition(
        ClimberConstants.rung1Position / (ClimberConstants.motorRadius * 2 * Math.PI));
    inputs.climberHeight = climberSimMotor.getTorqueCurrent() / 10;
    inputs.currentVoltage = climberSimMotor.getMotorVoltage();
  }

  @Override
  public void setTargetHeight(double height) {}

  @Override
  public void stopClimb() {}

  @Override
  public double getCurrentHeight() {
    return 0;
  }

  @Override
  public double getCurrentVoltage() {
    return 0;
  }
}
