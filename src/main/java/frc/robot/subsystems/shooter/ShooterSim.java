package frc.robot.subsystems.shooter;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.constants.ShooterConstants;

public class ShooterSim {
  private final FlywheelSim flySim =
      new FlywheelSim(
          LinearSystemId.identifyVelocitySystem(ShooterConstants.kV, ShooterConstants.kA),
          DCMotor.getKrakenX60(1),
          3.0);

  private final Mechanism2d mechCanvas = new Mechanism2d(3, 3);
  private final MechanismRoot2d root = mechCanvas.getRoot("pivot", 1.5, 1.5);
  MechanismLigament2d flywheelVis = root.append(new MechanismLigament2d("flywheel", 1, 0));

  public ShooterSim() {
    SmartDashboard.putData("Flywheel Stuff", mechCanvas);
  }

  public void periodic() {
    flySim.update(0.20);
    flySim.setAngularVelocity(ShooterSubsystem.getTargetVelocity());
    flywheelVis.setAngle(
        flywheelVis.getAngle() + Math.toDegrees(flySim.getAngularVelocityRadPerSec()));
    SmartDashboard.putNumber("Flywheel Velocity", flySim.getAngularVelocityRadPerSec());
  }
}
