package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class ClimberSubsystem extends SubsystemBase {

  private final ClimberIO climberIO;
  private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();
  //  private final Servo climbServo1 = new Servo(1);
  //  private final Servo climbServo2 = new Servo(2);
  //  private final Servo baseServo1 = new Servo(3);
  //  private final Servo baseServo2 = new Servo(4);
  DigitalInput L1Switch = new DigitalInput(0);

  public ClimberSubsystem(ClimberIO climberIO) {
    this.climberIO = climberIO;
  }

  @Override
  public void periodic() {
    climberIO.updateInputs(inputs);
    Logger.processInputs("Climber/ClimbingInputs", inputs);

    if (inputs.climberVoltage > 12) {
      climberIO.setVoltage(-Math.abs(inputs.climberVoltage - 10));
    } else if (inputs.climberVoltage < -12) {
      climberIO.setVoltage(-Math.abs(inputs.climberVoltage + 10));

      if (L1Switch.get()){
        climberIO.stopClimb();
      }
    }
  }

  /* ================= TELEMETRY ================= */

  public double getClimbHeight() {
    return inputs.climberHeight;
  }

  public void setVoltageSupplied(double voltageSupplied) {
    climberIO.setVoltage(voltageSupplied);
  }

  public void setClimbServoPosition(double servoPosition) {
    //    climbServo1.set(servoPosition);
    //    climbServo2.set(servoPosition);
  }

  public void setBaseServoPosition(double servoPosition) {
    //    baseServo1.set(servoPosition);
    //    baseServo2.set(servoPosition);
  }

  public void setTargetHeight(double desiredPosition) {
    climberIO.setTargetHeight(desiredPosition);
  }

  public void setkG(double kG){
    climberIO.setkG(kG);
  }

  public void resetPosition(){
    climberIO.resetPosition();
  }
}
