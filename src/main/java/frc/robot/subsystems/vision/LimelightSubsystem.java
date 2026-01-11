
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;

public class LimelightSubsystem extends SubsystemBase{

    double tx = LimelightHelpers.getTX(""); //offset in x direction'
    //tx>0 right
    //tx<0 left
    //best: tx=0

    double ty = LimelightHelpers.getTY(""); //offset in y direction
    double ta = LimelightHelpers.getTA(""); //target area 0-100%
    boolean hasTarget = LimelightHelpers.getTV("");// whether or not the camera has a target location
    double aprilTagID = 0;
    boolean useMegaTag2 = true; // turn into false if we decide to use MegaTag1
    // setting up the measured values of the camera to be set in the table//

    private final NetworkTable limelightTable;

    private NetworkTableEntry hasTarget;
    private NetworkTableEntry tx;
    private NetworkTableEntry ty;
    private NetworkTableEntry ta;
    private NetworkTableEntry aprilTagID;
    private NetworkTableEntry botpose; // CREATE A VARIABLE FOR BOTPOSE WHEN YOU FIGURE IT OUT.

    private double cameraHeight = 0.0;
    private double cameraAngle = 0.0;
    private double targetHeight = 0.0;

    public LimelightSubsystem() {
        limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
        LimelightHelpers.setPipelineIndex("", 0);  //setting the pipeline ID to 0 (goes from 0-10)
        LimelightHelpers.setLEDMode_PipelineControl(""); //LED set by the pipeline
    }

    // issue! Lots of the code relies on Drive Subsystem. Need to add code that mentions the current position of the robot
    @Override
    public void periodic() {
        if(useMegaTag2 == false) {
            LimelightHelpers.PoseEstimate positionEstimate = LimelightHelpers.getBotPoseEstimate("limelight");
        } else if (useMegaTag2 == true)
        {
            //LimelightHelpers.SetRobotOrientation("limelight", m_poseEstimator.getEstimatedPosition().getRotation().getDegrees(), 0, 0, 0, 0, 0);
            // above code needs Drive System for robot orientation.
            LimelightHelpers.PoseEstimate positionEstimate = LimelightHelpers.getBotPoseEstimate_MegaTag2("limelight");
}
        // need read april tag
        //pose estimator adjustment
        //if more features, you should need augular velocity (wait for gyro)
    //output = kp propotional to error ;  error relates to tx