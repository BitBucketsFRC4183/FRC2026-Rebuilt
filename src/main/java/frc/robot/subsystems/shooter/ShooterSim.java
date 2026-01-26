package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
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

  private final TalonFX m_motor = new TalonFX(1);
  private final TalonFXSimState m_motorSim = m_motor.getSimState();

  public ShooterSim() {
    SmartDashboard.putData("Flywheel Stuff", mechCanvas);
  }

  public void periodic() {
    // 1. Tell the TalonFX what the battery voltage is
    m_motorSim.setSupplyVoltage(RobotController.getBatteryVoltage());

    // 2. GET the voltage the TalonFX is trying to send to the motor
    double motorVoltage = m_motorSim.getMotorVoltage();

    // 3. APPLY that voltage to the physics model
    flySim.setInputVoltage(motorVoltage);

    // 4. STEP the physics model forward by 20ms
    flySim.update(0.02);

    // 5. UPDATE the TalonFX with the new simulated physics
    // Note: FlywheelSim returns mechanism velocity, but TalonFX wants ROTOR velocity.
    // Multiply by your gear ratio (e.g., if 1:1, use 1.0).
    double rotorVelocityRPS = flySim.getAngularVelocityRadPerSec() / (2 * Math.PI) * 3;
    m_motorSim.setRotorVelocity(rotorVelocityRPS);

    // 6. UPDATE the visualizer
    double deltaAngle = Math.toDegrees(flySim.getAngularVelocityRadPerSec() * 0.02);
    flywheelVis.setAngle(flywheelVis.getAngle() + deltaAngle);
  }
}
