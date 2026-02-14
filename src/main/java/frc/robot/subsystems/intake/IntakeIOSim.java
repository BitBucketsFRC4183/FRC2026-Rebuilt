package frc.robot.subsystems.intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.constants.IntakeConstants;

/**
 * Intake simulation IO for AdvantageScope.
 * Simulates motor velocity, current draw, and piston state.
 */
public class IntakeIOSim implements IntakeIO {

    // Simulated states
    private boolean piston1Extended = false;
    private boolean piston2Extended = false;
    private double motorOutput = 0.0;

    private double motorVelocityRPM = 0.0;
    private double motorCurrentAmps = 0.0;

    // Sim Constants
    private static final double FREE_SPEED_RPM = 6000.0;
    private static final double MAX_CURRENT_AMPS = 40.0;
    private static final double INTAKE_LOAD_CURRENT = 15.0;
    private static final double VELOCITY_RESPONSE = 12.0; // higher = snappier

    private double lastTimestamp = Timer.getFPGATimestamp();

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        double now = Timer.getFPGATimestamp();
        double dt = now - lastTimestamp;
        lastTimestamp = now;

        // Target velocity based on motor output
        double targetRPM = motorOutput * FREE_SPEED_RPM;

        motorVelocityRPM +=
                (targetRPM - motorVelocityRPM) * MathUtil.clamp(dt * VELOCITY_RESPONSE, 0.0, 1.0);

        // Current draw
        motorCurrentAmps =
                Math.abs(motorOutput) * MAX_CURRENT_AMPS;

        // Add intake load if extended and spinning inward
        if (piston1Extended && piston2Extended && motorOutput > 0.1) {
            motorCurrentAmps += INTAKE_LOAD_CURRENT;
        }

        //inputs
        inputs.motorVelocityRPM = motorVelocityRPM;
        inputs.motorCurrentAmps = motorCurrentAmps;
        inputs.primaryPistonExtended = piston1Extended;
        inputs.secondaryPistonExtended = piston2Extended;
    }

    @Override
    public void setMotorOutput(double percent) {
        motorOutput = MathUtil.clamp(percent, -1.0, 1.0);
    }

    @Override
    public void extend() {
        piston1Extended = true;
        piston2Extended = true;
    }

    @Override
    public void retract() {
        piston2Extended = false;
        piston1Extended = false;
    }
}
