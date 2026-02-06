package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ClimberConstants;
import org.littletonrobotics.junction.Logger;

public class ClimberSubsystem extends SubsystemBase {

  private final ClimberIO climberIO;
  private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();

  Servo servo1 = new Servo(1);
  Servo servo2 = new Servo(2);
  Servo servo3 = new Servo(3);
  Servo servo4 = new Servo(4);
  DigitalInput L1Switch = new DigitalInput(0);

  public ClimberSubsystem(ClimberIO climberIO) {
    this.climberIO = climberIO;
  }

  @Override
  public void periodic() {
    climberIO.updateInputs(inputs);
    Logger.processInputs("Climber/ClimbingInputs", inputs);
  }

  /* ================= ARM CONTROL ================= */

  public void moveClimbToGround() {
    climberIO.setTargetHeight(0);
    //    servo1.setAngle(0);
    //    servo2.setAngle(0);
    //    servo3.setAngle(0);
    //    servo4.setAngle(0);
    //    Commands.waitSeconds(2);
    //    servo1.set(0);
    //    servo2.set(0);
    //    servo3.set(0);
    //    servo4.set(0);
  }

  public void moveClimbToLevel1() {
    climberIO.setTargetHeight(ClimberConstants.rung1Position);
    //    servo1.set(.7);
    //    servo2.set(.7);
    //    servo3.set(.7);
    //    servo4.set(.7);
  }

  public void stopRise() {
    climberIO.stopClimb();
  }
  /* ================= TELEMETRY ================= */

  public double getClimbHeight() {
    return inputs.climberHeight;
  }
}
