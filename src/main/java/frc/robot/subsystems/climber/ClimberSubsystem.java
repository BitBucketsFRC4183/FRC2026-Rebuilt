package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase {

    private final ClimberIO io;
    private final ClimberIOInputs inputs = new ClimberIOInputs();

    public ClimberSubsystem(ClimberIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
    }

    /* ================= ARM CONTROL ================= */

    public void moveArmsToStowed() {
        io.setArmAngleDeg(ClimberConstants.ARM_STOWED_DEG);
    }

    public void moveArmsToExtended() {
        io.setArmAngleDeg(ClimberConstants.ARM_EXTENDED_DEG);
    }

    public void stopArms() {
        io.stopArm();
    }



    //Telemetry

    public double getArmAngleDeg() {
        return inputs.armAngleDeg;
    }

}
