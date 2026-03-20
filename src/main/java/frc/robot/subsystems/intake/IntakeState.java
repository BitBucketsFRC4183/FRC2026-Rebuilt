package frc.robot.subsystems.intake;

public enum IntakeState {
  STOWED, // Retracted, motor off
  DEPLOYED, // Extended, motor off
  INTAKING, // Extended, pulling fuel in
  RUN_STOWED, // Runs while stowed
  OUTTAKING, // Extended, pushing fuel out
  HOLD, // Holds after motor runs, to prevent state machine conflicts
  HOLD_STOWED, //Holds if stowed
}
