package frc.robot.subsystems.shooter;

public interface ShooterIO {
    void setSpeed(double targetSpeed);
    void stopMotor();
    boolean speedReached(double targetSpeed);
}
