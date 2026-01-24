package frc.robot.subsystems.intake;

public enum IntakeState {
    STOWED,     // Retracted, motor off
    DEPLOYED,   // Extended, motor off
    INTAKING,   // Extended, pulling fuel in
    OUTTAKING,  // Extended, pushing fuel out
    HOLDING     // Extended, low power hold
}
