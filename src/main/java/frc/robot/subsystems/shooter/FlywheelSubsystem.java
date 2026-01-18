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

public void setTargetHoodAngle(double distance) {
    double targetFlywheelRPM = 3000;
    //ill change later i j chose a value for now
    double launchVelocity = (targetFlywheelRPM * 2 * Math.PI / 60) * topFlywheelRadius;
    double verticalDistance = ShooterConstants.hubHeight - ShooterConstants.shooterHeight;
    //initialize velocity squared - physics variables
    double v0squared = Math.pow(v0, 2);
    double v0fourth = Math.pow(v0,4);

    double targetHoodAngle = ShooterConstants.minHoodAngle;
    //change accuracy for min and max hood angle later
    for ( double angle = ShooterConstants.minHoodAngle; angle <= ShooterConstants.maxHoodAngle; angle += 0.01){
        double launchVelX = launchVelocity * Math.cos(angle);
        double launchVelY = launchVelocity * Math.sin(angle);

        double discriminant = Math.pow(launchVelY, 2) + 2 * ShooterConstants.gravity * verticalDistance;
//discriminant = launch energy - energy needed for dist + height - theory
        if (discriminant >=0){
            double airtime = (launchVelY + Math.sqrt(discriminant))/ShooterConstants.gravity;

            double distanceAchieved = launchVelX * airtime;

            if(Math.abs(distnaceAchieved - distance) <= ShooterConstants.error){
                targetHoodAngle = angle;
            }
        }
setFlywheelSpeed(targetFlywheelRPM);
    }
//insert code for setting hood angle stuff
private void setHoodAngle (double angle){
        angle = Math.max((ShooterConstants.minHoodAngle, Math.min(ShooterConstants.maxHoodAngle, angle)));
//finish it after figuring out how to
    }
private void setFlywheelSpeed (double rpm){
        double controllerUnits = rpm /2/Math.PI * 60
                //check the math again
        m_topPID.setReference(controllerUnits, ControlType.kVelocity);
        m_bottomPID.setReference(controllerUnits, ControlType.kVelocity);
    }
}

    public boolean targetReached() {
        double tolerance = 50.0;
        return Math.abs(m_topEncoder.getVelocity() - topTargetRPM) < tolerance && Math.abs(m_bottomEncoder.getVelocity() - bottomTargetRPM) < tolerance;
    }
}
