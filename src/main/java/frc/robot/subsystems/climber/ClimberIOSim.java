package frc.robot.subsystems.climber;

import com.ctre.phoenix6.hardware.core.CoreTalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ClimberIOSim implements ClimberIO {
  CoreTalonFX coreTalonFX = new CoreTalonFX(1);
  public final TalonFXSimState climberSimMotor = coreTalonFX.getSimState();

  private final Mechanism2d climberCanvas = new Mechanism2d(3, 3);
  private final MechanismRoot2d climberRoot = climberCanvas.getRoot("pivot", 1.5, 1.5);
  MechanismLigament2d climberModel =
      climberRoot.append(new MechanismLigament2d("climberMotor", 1, 0));

  public ClimberIOSim() {
    SmartDashboard.putData("Climber Data", climberCanvas);
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    climberModel.setLength(67);
    inputs.climberHeight = climberSimMotor.getTorqueCurrent() / 5;
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
