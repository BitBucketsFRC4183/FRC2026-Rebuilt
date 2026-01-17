package frc.robot.subsystems.shooter;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkRelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import frc.robot.constants.ShooterConstants;

public class FlywheelSubsystem {
    private final SparkMax m_topFlywheel = new SparkMax(ShooterConstants.topFlywheelID, MotorType.kBrushless);
    private final SparkMax m_bottomFlywheel = new SparkMax(ShooterConstants.bottomFlywheelID, MotorType.kBrushless);

    private final SparkRelativeEncoder m_topFlywheelEncoder = m_topFlywheel.getEncoder();
    private final SparkRelativeEncoder m_bottomFlywheelEncoder = m_bottomFlywheel.getEncoder();

    private final SparkClosedLoopController m_topPID = m_topFlywheel.getClosedLoopController();
    private final SparkClosedLoopController m_bottomPID = m_bottomFlywheel.getClosedLoopController();

    double topTargetFlywheelVelocity, bottomTargetFlywheelVelocity;
    //In meters
    double hubHeight, shooterHeight, topFlywheelRadius, bottomFlywheelRadius;

    public FlywheelSubsystem() {
        SparkMaxConfig config = new SparkMaxConfig();
        config.smartCurrentLimit(40)
                .idleMode(IdleMode.kCoast);

        config.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .p(ShooterConstants.kP)
                .i(ShooterConstants.kI)
                .d(ShooterConstants.kD)
                .velocityFF(ShooterConstants.kFF)
                .outputRange(ShooterConstants.kMinOutput, ShooterConstants.kMaxOutput);

        //Convert all American Units to Metric
        hubHeight = ShooterConstants.hubHeight / 39.37;
        shooterHeight = ShooterConstants.shooterHeight / 39.37;
        topFlywheelRadius = ShooterConstants.topFlywheelRadius / 39.37;
        bottomFlywheelRadius = ShooterConstants.bottomFlywheelRadius / 39.37;

        m_topFlywheel.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        m_bottomFlywheel.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    //BACKUP PLAN
    public void startFlywheels() {

    }

    //The Distance inputted should be the distance of the bot from the "center" of the hub
    public void setTargetFlywheelVelocity(double distance) {
        topTargetFlywheelVelocity = 0;
        for(double distanceAchieved = 0; distanceAchieved <= distance + ShooterConstants.error && distanceAchieved >= distance - ShooterConstants.error; topTargetFlywheelVelocity++) {
            double yVelocity = topTargetFlywheelVelocity * topFlywheelRadius;
            double h = ShooterConstants.hubHeight - ShooterConstants.shooterHeight;
            double airTime = -yVelocity + (Math.sqrt(Math.pow(yVelocity, 2) - 2 * ShooterConstants.gravity * h)) / 2 / h;
            distanceAchieved = airTime * topTargetFlywheelVelocity * topFlywheelRadius * Math.tan(ShooterConstants.shooterAngle);
            if(topTargetFlywheelVelocity > ShooterConstants.maxRPM) {break;}
        }
        m_topPID.setReference(topTargetFlywheelVelocity / 2 / Math.PI * 60, ControlType.kVelocity);

        //Makes both Flywheels have same tangential velocity
        bottomTargetFlywheelVelocity = topFlywheelRadius * bottomFlywheelRadius / topFlywheelRadius;
        m_bottomPID.setReference(bottomFlywheelRadius / 2 / Math.PI * 60, ControlType.kVelocity);
    }

    public boolean targetReached() {
        double tolerance = 50.0;
        return Math.abs(m_topEncoder.getVelocity() - topTargetRPM) < tolerance && Math.abs(m_bottomEncoder.getVelocity() - bottomTargetRPM) < tolerance;
    }
}
