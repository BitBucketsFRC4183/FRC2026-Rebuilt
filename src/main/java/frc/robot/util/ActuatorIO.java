package frc.robot.util;


/** IO interface for the actuator sub */
public interface ActuatorIO {

    void updateInputs(ActuatorIOInputs inputs);

    void setActuatorPercent(double percent);
    void stopActuator();

}
