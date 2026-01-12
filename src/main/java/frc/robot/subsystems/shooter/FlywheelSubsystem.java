package frc.robot.subsystems.shooter;

public class FlywheelSubsystem {
    private final SparkMax m_topFlywheel = new SparkMax(ShooterConstants.topFlywheelID, MotorType.kBrushless);
    private final SparkMax m_bottomFlywheel = new SparkMax(ShooterConstants.bottomFlywheelID, MotorType.kBrushless);

    private final RelativeEncoder m_topFlywheelEncoder = m_topFlywheel.getEncoder();
    private final RelativeEncoder m_bottomFlywheelEncoder = m_bottomFlywheel.getEncoder();

    double targetTopFlywheelVelocity, targetBottomFlywheelVelocity;
    //In meters
    double hubHeight, shooterHeight;

    public FlywheelSubsystem() {
        SparkMaxConfig config = new SparkMaxConfig();
        config.smartCurrentLimit(40);
        config.idleMode(IdleMode.kCoast);
        m_topFlywheel.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_bottomFlywheel.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        //Sets up constants for the PID used to control the Flywheels
        m_pidController.setP(ShooterConstants.kP);
        m_pidController.setI(ShooterConstants.kI);
        m_pidController.setD(ShooterConstants.kD);
        m_pidController.setFF(ShooterConstants.kFF);
        m_pidController.setOutputRange(ShooterConstants.kMinOutput, ShooterConstants.kMaxOutput);

        hubHeight = ShooterConstants.hubHeight / 39.37;
        shooterHeight = ShooterConstants.shooterHeight / 39.37;
        //startFlywheels();
    }

    //BACKUP PLAN
    public void startFlywheels() {
        m_topFlywheel.setVoltage(0.8);
        m_bottomFlywheel.setVoltage(0.8);
    }

    public void setFlywheelVelocity(double distance) {
        double time = 1;
        double xVelocity = distance / time;
        double yVelocity = ShooterConstants.gravity * time / 2;

        //In radians/sec
        //Arbitrary Value (change if not fast enough)
        targetBottomFlywheelVelocity = 30;

        targetTopFlywheelVelocity = (2 *  xVelocity - targetBottomFlywheelVelocity * ShooterConstants.flywheelRadius) / ShooterConstants.flywheelRadius;
        targetBottomFlywheelVelocity = (2 * yVelocity + targetTopFlywheelVelocity * ShooterConstants.flywheelRadius) / ShooterConstants.flywheelRadius;

        m_topFlywheel.setSetpoint(targetTopFlywheelVelocity / 2 / Math.PI * 60, ControlType.kVelocity);
        m_bottomFlywheel.setSetpoint(targetBottomFlywheelVelocity / 2 / Math.PI * 60, ControlType.kVelocity);
    }

    public boolean velocityReached() {
        return false;
    }
}
