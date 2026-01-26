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

    public void moveClimbToGround() {
        io.setTargetHeight(0);
    }

    public void moveClimbToLevel1() {
        io.setTargetHeight(ClimberConstants.rung1Position);
    }

    public void stopArms() {
        io.stopClimb();
    }
    /* ================= TELEMETRY ================= */

    public double getClimbHeight() {
        return inputs.climberHeight;
    }

}
