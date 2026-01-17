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

    private final SparkRelativeEncoder m_topEncoder = m_topFlywheel.getEncoder();
    private final SparkRelativeEncoder m_bottomEncoder = m_bottomFlywheel.getEncoder();

    private final SparkClosedLoopController m_topPID = m_topFlywheel.getClosedLoopController();
    private final SparkClosedLoopController m_bottomPID = m_bottomFlywheel.getClosedLoopController();

    double topTargetFlywheelVelocity, bottomTargetFlywheelVelocity;
    //In meters
    double hubHeight, shooterHeight, hubDistance, topFlywheelRadius, bottomFlywheelRadius;

    public FlywheelSubsystem() {
        SparkMaxConfig config = new SparkMaxConfig();
        config.smartCurrentLimit(40)
                .idleMode(IdleMode.kCoast);

        //For PID and Feed Forward
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
        hubDistance = ShooterConstants.distanceFromCenter / 39.37;

        //Configures both Flywheels, inverts the bottom Flywheel
        m_topFlywheel.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        config.inverted(true);
        m_bottomFlywheel.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    public void setTargetFlywheelVelocity(double distance) {
        //Distance from April-Tag + Distance away from center of the Hub
        distance += hubDistance;
        topTargetFlywheelVelocity = 0;
        //Computationally heavy, TODO: Use a different method to find RPM
        for(double distanceAchieved = 0; distanceAchieved <= distance + ShooterConstants.error && distanceAchieved >= distance - ShooterConstants.error; topTargetFlywheelVelocity+= 2) {
            //Calculating tangential velocity
            double yVelocity = topTargetFlywheelVelocity * topFlywheelRadius * Math.tan(ShooterConstants.shooterAngle);
            //Using Kinematics to calculate RPM to launch the ball, hopefully air resistance is negligible lol
            double h = ShooterConstants.hubHeight - ShooterConstants.shooterHeight;
            double airTime = -yVelocity + (Math.sqrt(Math.pow(yVelocity, 2) - 2 * ShooterConstants.gravity * h)) / 2 / h;
            distanceAchieved = airTime * topTargetFlywheelVelocity * topFlywheelRadius * Math.tan(ShooterConstants.shooterAngle);
            //If it reaches this statement, the bot is too far from the hub to make it in
            if(topTargetFlywheelVelocity > ShooterConstants.maxRPM / 2 / Math.PI * 60) {break;}
        }

        m_topPID.setReference(topTargetFlywheelVelocity / 2 / Math.PI * 60, ControlType.kVelocity);
        //Makes both Flywheels have same tangential velocity
        bottomTargetFlywheelVelocity = topTargetFlywheelVelocity * bottomFlywheelRadius / topFlywheelRadius;
        m_bottomPID.setReference(bottomFlywheelRadius / 2 / Math.PI * 60, ControlType.kVelocity);
        //TODO: implement adjustable hood
    }

    //Emergency Stop?
    public void stop() {
        m_topFlywheel.stopMotor();
        m_bottomFlywheel.stopMotor();
    }

    //When Triggered Pressed, wait until true, then use motor to fire all the balls in storage
    //Operator is gonna have one button, and they don't even have to hold it down :sob:
    public boolean targetReached() {
        double tolerance = 50.0;
        return Math.abs(m_topEncoder.getVelocity() - topTargetRPM) < tolerance && Math.abs(m_bottomEncoder.getVelocity() - bottomTargetRPM) < tolerance;
    }
}