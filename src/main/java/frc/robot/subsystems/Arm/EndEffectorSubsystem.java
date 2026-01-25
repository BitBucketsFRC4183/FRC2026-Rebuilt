package frc.robot.subsystems.Arm;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.EndEffectorConstants;

public class EndEffectorSubsystem extends SubsystemBase {
    private final EndEffectorIO endEffector;
    private final EndEffectorIO.EndEffectorInputs inputs = new EndEffectorIO.EndEffectorInputs();

    public EndEffectorSubsystem(EndEffectorIO endEffector) {
        this.endEffector = endEffector;
    }

    public void disable() {
        endEffector.disable();//rotate grippers for better hold
        endEffector.setIsOpen(false);
    }

    public void setEndVoltage(double volts) {
        endEffector.setEndVoltage(volts);
    }

    public void setEndVelocity(double velocity) {
        endEffector.setEndVelocity(velocity);
    }

    @Override
    public void periodic() {
        endEffector.updateInputs(inputs);
    }
}

