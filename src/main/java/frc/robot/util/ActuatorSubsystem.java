package frc.robot.util;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.HopperConstants;

public class ActuatorSubsystem extends SubsystemBase {

    private final ActuatorIO io;
    private final ActuatorIOInputs inputs = new ActuatorIOInputs();

    public ActuatorSubsystem(ActuatorIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
    }

    //Conveyor
    public void runActuatorForward() {
        io.setActuatorPercent(ActuatorConstants.ACTUATOR_FORWARD_PERCENT);
    }

    //Run in Reverse
    public void runActuatorReverse() {
        io.setActuatorPercent(ActuatorConstants.ACTUATOR_REVERSE_PERCENT);
    }

    public void stopActuator() {
        io.stopActuator();
    }

    //Telemetry

    public double getActuatorOutput() {
        return inputs.actuatorAppliedOutput;
    }
}
