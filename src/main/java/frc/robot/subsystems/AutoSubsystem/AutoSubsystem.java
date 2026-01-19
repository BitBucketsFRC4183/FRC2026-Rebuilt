package frc.robot.subsystems.AutoSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.DriveSubsystem.DriveSubsystem;

public class AutoSubsystem extends SubsystemBase {
    private final DriveSubsystem drive;
    private final ShooterSubsystem shooter;
    private final ClimberSubsystem climber;




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

        public AutoRoutine MidShootTowerl1(){

                AutoRoutine MidShootTowerL1 =
                        autoFactory.newRoutine("MidShootTowerL1");
        //midstarttomid is start at mid move back shoot and turn to climb
              //initialize 1
                AutoTrajectory MidStarttoMid =
                        MidShootTowerL1.trajectory("MidStarttoMid");
                //initialize 2
                AutoTrajectory MidStarttoTower =
                        MidShootTowerL1.trajectory("MidStarttoTower");

                MidShootTowerL1.active().onTrue(
                    Commands.sequence(
                            Commands.print("Started" + "MidShootTowerL1" + "routine:)")
                            MidStarttoMid().resetOdometry()
                            MidStarttoMid.cmd()
                    )
            );

                MidStarttoMid.active();
                MidStarttoMid.done().onTrue(shoot()).andThen(MidStarttoTower.cmd());

                MidStarttoTower.active().onTrue(climb());

                return MidShootTowerL1;

            }

            public AutoRoutine TopShootTowerL1(){
            //move from top to mid shoot then go mid to tower
                    AutoRoutine TopShootTowerL1 =
                            autoFactory.newRoutine("TopShootTowerL1");
                    //intialize 1
                    AutoTrajectory TopStartToMid=
                            TopShootTowerL1.trajectory("TopStartToMid");
                    //initialize 2
                AutoTrajectory MidtoTower=
                        TopShootTowerL1.trajectory("MidtoTower");

                TopShootTowerL1.active().onTrue(

                        Commands.sequence(
                                Commands.print("Started" + "TopStartToMid" + "routine:)")
                                TopStartToMid.resetOdometry()
                                TopStartToMid.cmd();
                        )

                        TopStartToMid.active();
                        TopStartToMid.done().onTrue(shoot().andThen(MidStarttoTower.cmd()));
                        //note to navya - check the last sequential stuff and check which one is right
                        MidStarttoTower.active().onTrue(climb());

                return TopShootTowerL1;
            }
}
