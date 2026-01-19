package frc.robot.subsystems.shooter;

import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkMax;

//Deprecated
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkRelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;

import frc.robot.constants.ShooterConstants;

public class ShooerSubsystem {
    private final SparkMax m_topFlywheel = new SparkMax(ShooterConstants.topFlywheelID, MotorType.kBrushless);
    private final SparkMax m_bottomFlywheel = new SparkMax(ShooterConstants.bottomFlywheelID, MotorType.kBrushless);

    private final SparkRelativeEncoder m_topEncoder = (SparkRelativeEncoder) m_topFlywheel.getEncoder();
    private final SparkRelativeEncoder m_bottomEncoder = (SparkRelativeEncoder) m_bottomFlywheel.getEncoder();

    private final SparkClosedLoopController m_topPID = m_topFlywheel.getClosedLoopController();
    private final SparkClosedLoopController m_bottomPID = m_bottomFlywheel.getClosedLoopController();

    double topTargetFlywheelVelocity, bottomTargetFlywheelVelocity;
    //In meters
    double hubHeight, shooterHeight, hubDistance, topFlywheelRadius, bottomFlywheelRadius;

    public ShooerSubsystem() {
        SparkMaxConfig config = new SparkMaxConfig();
        config.smartCurrentLimit(40)
                .idleMode(IdleMode.kCoast);

        //For PID and Feed Forward
        config.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .p(ShooterConstants.kP)
                .i(ShooterConstants.kI)
                .d(ShooterConstants.kD)
                .outputRange(ShooterConstants.kMinOutput, ShooterConstants.kMaxOutput);
        config.closedLoop.feedForward
                .kS(ShooterConstants.kS)
                .kV(ShooterConstants.kV)
                .kA(ShooterConstants.kA)
                .kCosRatio(ShooterConstants.cosRatio);

        //Convert all American Units to Metric
        hubHeight = ShooterConstants.hubHeight / 39.37;
        shooterHeight = ShooterConstants.shooterHeight / 39.37;
        topFlywheelRadius = ShooterConstants.topFlywheelRadius / 39.37;
        bottomFlywheelRadius = ShooterConstants.bottomFlywheelRadius / 39.37;
        hubDistance = ShooterConstants.distanceFromCenter / 39.37;

        //Configures both Flywheels, inverts the bottom Flywheel
        //Deprecated again lol
        m_topFlywheel.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        config.inverted(true);
        m_bottomFlywheel.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    public void setTargetFlywheelVelocity(double distance) {
        //Method for setting flywheel speeds assuming constant hood angle
        //Distance from April-Tag + Distance away from center of the Hub
        distance += hubDistance;
        double radius;
        //Uses the radius of the smallest fly wheel (so that we do not go over the RPM limit)
        if(topFlywheelRadius > bottomTargetFlywheelVelocity) {radius = bottomFlywheelRadius;}
        else {radius = topFlywheelRadius;}
        double targetFlywheelVelocity = 0;
        //Computationally heavy, TODO: Use a different method to find RPM
        for(double distanceAchieved = 0; distanceAchieved >= distance; targetFlywheelVelocity += 2) {
            //Calculating tangential velocity
            double yVelocity = targetFlywheelVelocity * radius * Math.sin(Math.toRadians(ShooterConstants.shooterAngle));
            //Using Kinematics to calculate RPM to launch the ball, hopefully air resistance is negligible lol
            double h = ShooterConstants.hubHeight - ShooterConstants.shooterHeight;
            double airTime = -yVelocity + (Math.sqrt(Math.pow(yVelocity, 2) - 2 * ShooterConstants.gravity * h)) / ShooterConstants.gravity;
            distanceAchieved = airTime * targetFlywheelVelocity * radius * Math.cos(Math.toRadians(ShooterConstants.shooterAngle));
            //If it reaches this statement, the bot is too far from the hub to make it in or the shooterAngle is not appropriate
            if(targetFlywheelVelocity > ShooterConstants.maxRPM * 2 * Math.PI / 60) {break;}
        }

        if(topFlywheelRadius > bottomTargetFlywheelVelocity) {
            bottomTargetFlywheelVelocity = targetFlywheelVelocity;
            m_bottomPID.setSetpoint(bottomFlywheelRadius / 2 / Math.PI * 60, ControlType.kVelocity);
            topTargetFlywheelVelocity = bottomTargetFlywheelVelocity * bottomFlywheelRadius / topFlywheelRadius;
            m_topPID.setSetpoint(topTargetFlywheelVelocity / 2 / Math.PI * 60, ControlType.kVelocity);
        }
        else {
            topFlywheelRadius = targetFlywheelVelocity;
            m_topPID.setSetpoint(topTargetFlywheelVelocity / 2 / Math.PI * 60, ControlType.kVelocity);
            bottomTargetFlywheelVelocity = topTargetFlywheelVelocity * topFlywheelRadius / bottomFlywheelRadius;
            m_bottomPID.setSetpoint(bottomFlywheelRadius / 2 / Math.PI * 60, ControlType.kVelocity);
        }
    }

    //Emergency Stop?
    public void stop() {
        m_topFlywheel.stopMotor();
        m_bottomFlywheel.stopMotor();
    }

    //When Triggered Pressed, wait until true, then use motor to fire all the balls in storage
    //Operator is going to have one button, and they don't even have to hold it down :sob:
    public boolean targetReached() {
        double tolerance = 50.0;
        return Math.abs(m_topEncoder.getVelocity() - topTargetFlywheelVelocity / 2 / Math.PI * 60) < tolerance && Math.abs(m_bottomEncoder.getVelocity() - bottomTargetFlywheelVelocity / 2 / Math.PI * 60) < tolerance;
    }
}