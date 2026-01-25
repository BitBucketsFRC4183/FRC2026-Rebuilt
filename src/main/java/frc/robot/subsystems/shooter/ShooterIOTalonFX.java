package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.constants.ShooterConstants;

public class ShooterIOTalonFX implements ShooterIO {
  private final TalonFX finalFlywheel = new TalonFX(ShooterConstants.flywheelID);
  private final TalonFX intermediateMotor = new TalonFX(ShooterConstants.intermediateID);
  private final VelocityVoltage target = new VelocityVoltage(0);

  private final FlywheelSim flySim =
      new FlywheelSim(
          LinearSystemId.identifyVelocitySystem(ShooterConstants.kV, ShooterConstants.kA),
          DCMotor.getKrakenX60(1),
          3.0);

  private final Mechanism2d mechCanvas = new Mechanism2d(3, 3);
  private final MechanismRoot2d root = mechCanvas.getRoot("pivot", 1.5, 1.5);
  MechanismLigament2d flywheelVis = root.append(new MechanismLigament2d("flywheel", 1, 0));

  public ShooterIOTalonFX() {
    SmartDashboard.putData("Flywheel", mechCanvas);
    TalonFXConfiguration motorConfig = new TalonFXConfiguration();
    motorConfig.MotorOutput.Inverted =
        ShooterConstants.flywheelInverted
            ? com.ctre.phoenix6.signals.InvertedValue.Clockwise_Positive
            : com.ctre.phoenix6.signals.InvertedValue.CounterClockwise_Positive;

    // Hopefully can make the wind uptime for the flywheel faster
    motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    // PID and FF Configs
    Slot0Configs slot0 = motorConfig.Slot0;
    slot0.kP = ShooterConstants.kP;
    slot0.kI = ShooterConstants.kI;
    slot0.kD = ShooterConstants.kD;
    slot0.kA = ShooterConstants.kA;
    slot0.kV = ShooterConstants.kV;
    slot0.kS = ShooterConstants.kS;

    // Current Limits
    CurrentLimitsConfigs currentConfig = new CurrentLimitsConfigs();
    currentConfig.SupplyCurrentLimitEnable = true;
    currentConfig.SupplyCurrentLimit = 40;
    currentConfig.StatorCurrentLimitEnable = true;
    currentConfig.StatorCurrentLimit = 40;

    finalFlywheel.getConfigurator().apply(motorConfig);
    finalFlywheel.getConfigurator().apply(currentConfig);

    // Intermediate Motor Configs
    motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    intermediateMotor.getConfigurator().apply(motorConfig);
    intermediateMotor.getConfigurator().apply(currentConfig);
  }

  @Override
  public void setSpeed(double targetSpeed) {
    // Convert Radians / s to Rotations / s
    double targetRPS = targetSpeed / 2 / Math.PI;
    // Please be the same radius
    finalFlywheel.setControl(target.withVelocity(targetRPS));
  }

  @Override
  public void startIntermediateMotors() {
    intermediateMotor.setControl(target.withVelocity(ShooterConstants.intermediateSpeed));
  }

  @Override
  public void stopMotor() {
    finalFlywheel.stopMotor();
    intermediateMotor.stopMotor();
  }

  @Override
  public boolean speedReached(double targetSpeed) {
    double currentTopFlywheelVelocity = finalFlywheel.getVelocity().getValueAsDouble();
    return currentTopFlywheelVelocity < targetSpeed + ShooterConstants.tolerance
        && currentTopFlywheelVelocity > targetSpeed - ShooterConstants.tolerance;
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    inputs.appliedFlywheelOutput = finalFlywheel.getDutyCycle().getValueAsDouble();
    inputs.appliedIntermediateOutput = intermediateMotor.getDutyCycle().getValueAsDouble();
    inputs.flywheelVelocity = finalFlywheel.getVelocity().getValueAsDouble();
    inputs.intermediateVelocity = intermediateMotor.getVelocity().getValueAsDouble();
  }

  @Override
  public void simulationPeriodic() {
    // --- CTRE PHOENIX 6 SIMULATION LOGIC ---

    // A. Get the simulation state of the TalonFX
    var talonSimState = finalFlywheel.getSimState();

    // B. Set the Supply Voltage (important so the Talon knows it has power)
    talonSimState.setSupplyVoltage(RobotController.getBatteryVoltage());

    // C. Update Physics Engine
    // Get the voltage the Talon WANTS to output and feed it to the physics sim
    flySim.setInput(talonSimState.getMotorVoltage());
    flySim.update(0.02); // 20ms loop time
    flySim.setAngularVelocity(50);

    // D. "Close the Loop"
    // We must tell the TalonFX how fast the sim thinks it is spinning.
    // WPILib uses Radians/Sec, Phoenix 6 uses Rotations/Sec.
    // Formula: Rad/s / 2Pi = Rotations/Sec
    double simRPS = flySim.getAngularVelocityRadPerSec() / (2 * Math.PI);
    talonSimState.setRotorVelocity(simRPS);

    // E. Update Visualization (Mechanism2d)
    // Convert Rad/s to Deg/s for the visual rotation
    flywheelVis.setAngle(
        flywheelVis.getAngle() + flySim.getAngularVelocityRadPerSec() * 0.02 * (180 / Math.PI));
    SmartDashboard.putNumber("Flywheel RPM", flySim.getAngularVelocityRPM());
  }
}

// Aidan's Social Security Number: 621 79 0241
