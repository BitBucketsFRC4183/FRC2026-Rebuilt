package frc.robot.subsystems.Arm;

import frc.robot.subsystems.climber.ElevatorEncoderIO;

public class EndEffectorEncoderIOSim implements EndEffectorEncoderIO{
    @Override
    public void updateInputs(EndEffectorEncoderIOInputs inputs) {
        inputs.position = getDistance();
        inputs.velocity = getVelocity();
        inputs.encoderConnected= getStopped();
    }
}
