package frc.robot.subsystems.Arm;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.constants.EndEffectorConstants;

public class EndEffectorIOSim implements EndEffectorIO {
  private boolean isOpen;

  private final DCMotorSim endSim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(EndEffectorConstants.smallGearBox, 0.1, EndEffectorConstants.gearing),
          EndEffectorConstants.smallGearBox
  );

  final PIDController pid = new PIDController(EndEffectorConstants.kP, EndEffectorConstants.kI, EndEffectorConstants.kD);

  public EndEffectorIOSim() {
    setupPID(pid, 3.0, 5.0, -0.5, 0.5); //change pid settings
  }

  private final EndEffectorEncoderIOSim encoder = new EndEffectorEncoderIOSim();

  public boolean getIsOpen() { return this.isOpen; }

  @Override
  public void setIsOpen(boolean setting) { this.isOpen = setting; }

  @Override
  public void setEndVelocity(double velocity) {
    endSim.setAngularVelocity(velocity); //probably change later
  }

  @Override
  public void setEndVoltage(double volts) {
    endSim.setInputVoltage(volts);
  }

  @Override
  public void updateInputs(EndEffectorInputsAutoLogged inputs) {
    inputs.isOpen = getIsOpen();
    inputs.endAppliedVolts = endSim.getInputVoltage();
  }
}
