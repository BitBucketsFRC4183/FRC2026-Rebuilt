package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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
    //    if (climberIO.getCurrentHeight() >= ClimberConstants.maxHeight) {
    //      climberIO.setVoltage(0);
    //      Commands.waitSeconds(2);
    //      climberIO.setVoltage(-2);
    //    }
    //    if (climberIO.getCurrentHeight() <= ClimberConstants.minHeight) {
    //      climberIO.setVoltage(0);
    //      Commands.waitSeconds(2);
    //      climberIO.setVoltage(2);
    //    }
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

  public void stopRise() {
    climberIO.stopClimb();
  }
  /* ================= TELEMETRY ================= */

  public double getClimbHeight() {
    return inputs.climberHeight;
  }

  public void moveClimbToLevel1() {
    // climberIO.setTargetHeight(ClimberConstants.rung1Position);
    climberIO.setTargetHeight(0);
    //    servo1.set(.7);
    //    servo2.set(.7);
    //    servo3.set(.7);
    //    servo4.set(.7)
  }

  public void setVoltageSupplied(double voltageSupplied) {
    climberIO.setVoltage(voltageSupplied);
  }
}
