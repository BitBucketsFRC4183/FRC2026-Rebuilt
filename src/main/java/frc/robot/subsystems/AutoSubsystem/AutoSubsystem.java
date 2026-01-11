

public class AutoSubsystem extends SubsystemBase {
    private final DriveSubsystem drive;
    private final ShooterSubsystem shooter;
    private final ClimberSubsystem climber;



this.AutoSubsystem =new

    AutoSubsystem(driveSubsystem, elevatorSubsystem, armSubsystem);

    autoChooser =new

    AutoChooser();

    public AutoSubsystem(DriveSubsystem drive, ClimbSubsystem climber, ShooterSubystem shooter) {
        this.drive = drive;
        this.climber = climber;
        this.shooter = shooter;
        this.autoFactory = new AutoFactory(drive::getPose, drive::setPose, drive::followTrajectorySample, false, drive, trajectoryLogger());

    }

    public Command stop(){

    }
    public Command shoot(){

    }
    public Command climb(){

    }
}
