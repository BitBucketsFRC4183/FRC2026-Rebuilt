package frc.robot.subsystems.climber;

public interface ClimberIO {

    void updateInputs(ClimberIOInputs inputs);

    // Arms
    void setArmAngleDeg(double degrees);
    void stopArm();

    // Hooks
    void setHookPositionRotations(double rotations);
    void stopHooks();
}
