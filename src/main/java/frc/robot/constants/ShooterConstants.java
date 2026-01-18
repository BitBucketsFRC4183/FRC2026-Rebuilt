package frc.robot.constants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public final class ShooterConstants extends SubsystemBase {

    //Motor IDs
    public final static int topFlywheelID = 6;
    public final static int bottomFlywheelID = 7;

    //In Inches por favor
    public final static double topFlywheelRadius = 1;
    public final static double bottomFlywheelRadius = 2;

    //Arena/Robot Constants
    //Heights are in inches off the Arena floor
    public final static double hubHeight = 72;
    public final static double shooterHeight = 21.5;
    public final static double gravity = 9.81;
    public final static double shooterAngle = 45;
    public final static double maxRPM = 5000;

    //Allowed Error for (actual distance away vs distance from launching)
    public final static double error = 5;

    //PID Constants
    public final static double kP = 0.00001;
    public final static double kI = 0;
    public final static double kD = 0;
    public final static double kFF = 0.00015;
    public final static double kMaxOutput = 1.0;
    public final static double kMinOutput = -1.0;

    //hood constants
    public final static double minHoodAngle = 10.0;
    public final static double maxHoodAngle = 80.0;
    //change accuracy later


}
