package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ClimberConstants;
import org.littletonrobotics.junction.Logger;

public class ClimberSubsystem extends SubsystemBase {

  private final ClimberIOTalonFX climberIOTalonFX;
  private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();

  Servo servo1 = new Servo(1);
  Servo servo2 = new Servo(2);
  Servo servo3 = new Servo(3);
  Servo servo4 = new Servo(4);
  DigitalInput L1Switch = new DigitalInput(0);

  public ClimberSubsystem(ClimberIOTalonFX climberIOTalonFX) {
    this.climberIOTalonFX = climberIOTalonFX;
  }

  @Override
  public void periodic() {
    climberIOTalonFX.updateInputs(inputs);
    Logger.processInputs("Climber/ClimbingInputs", inputs);
  }

  /* ================= ARM CONTROL ================= */

  public void moveClimbToGround() {
    climberIOTalonFX.setTargetHeight(0);
    //    servo1.setAngle(0);
    //    servo2.setAngle(0);
    //    servo3.setAngle(0);
    //    servo4.setAngle(0);
    //    Commands.waitSeconds(2);
    //    servo1.set(0);
    //    servo2.set(0);
    //    servo3.set(0);
    //    servo4.set(0);
    inputs.climberHeight = climberIOTalonFX.getCurrentHeight();
    inputs.currentVoltage = climberIOTalonFX.getCurrentVoltage();
  }

  public void moveClimbToLevel1() {
    climberIOTalonFX.setTargetHeight(ClimberConstants.rung1Position);
    //    servo1.set(.7);
    //    servo2.set(.7);
    //    servo3.set(.7);
    //    servo4.set(.7);
    inputs.climberHeight = climberIOTalonFX.getCurrentHeight();
    inputs.currentVoltage = climberIOTalonFX.getCurrentVoltage();
  }

  public void stopRise() {
    climberIOTalonFX.stopClimb();
  }
  /* ================= TELEMETRY ================= */

  public double getClimbHeight() {
    return inputs.climberHeight;
  }
}
