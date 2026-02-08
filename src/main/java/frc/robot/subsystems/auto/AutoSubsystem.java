package frc.robot.subsystems.auto;


import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.config.RobotConfig;

import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import edu.wpi.first.wpilibj2.command.*;



import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class AutoSubsystem extends SubsystemBase {

  private final DriveSubsystem drive;
  private final ShooterSubsystem shooter;

  public AutoSubsystem(DriveSubsystem drive, ShooterSubsystem shooter) {
    this.drive = drive;
    this.shooter = shooter;
    configureAutoBuilder();
    registerNamedCommands();
  }

  //PathPlannerPath MidStarttoMid = PathPlannerPath.fromChoreoTrajectory("MidStarttoMid");
  //Setup PathPlanner

  private void configureAutoBuilder() {
            AutoBuilder.configureHolonomic(
                    drive::getPose,
                    drive::resetPose,
                    drive::getRobotRelativeSpeeds,
            drive::driveRobotRelative,
            new PPHolonomicDriveController(
                    new PIDConstants(5.0, 0.0, 0.0),
                    new PIDConstants(5.0, 0.0, 0.0)
            ),
            RobotConfig.fromGUISettings(),
            drive::shouldFlipPath,
            drive
    );
  }

  //Named Commands

  private void registerNamedCommands() {

    NamedCommands.registerCommand(
            "Stop",
            stop()
    );

    NamedCommands.registerCommand(
            "Shoot",
            shoot()
    );

    NamedCommands.registerCommand(
            "Climb",
            climb()
    );
  }

  // Auto Actions

  public Command stop() {
    return Commands.runOnce(drive::stop, drive);
  }

  public Command shoot() {
    return Commands.print("shooting!");
  }

  public Command climb() {
    return Commands.print("Climbing!");
    // replace with real climb command
  }

  //autoroutines
  /**TODO: NAVYA BELOW IS THE CORRECT WAY TO MAKE A PATHPLANNER COMMAND.
   * YOU NEED TO CHANGE ALL OF YOUR COMMANDS TO MATCH "goBottomToMid"'S FORMATTING
   * AND THEN YOU MUST TEST YOUR PATHS IN SIM!! PLEASSSEE!!!
   */

  public Command goBottomToShootPs() {
    PathPlannerPath path1;

    try {
      // Load the path from the deploy/pathplanner folder by name
      path1 = PathPlannerPath.fromPathFile("BottomStartToShootB");
    } catch (Exception e) {
      e.printStackTrace();
      return Commands.none();
    }

    // Create a path-following command using AutoBuilder
    return AutoBuilder.followPath(path1);
  }



  public Command goToptoShooterPs(){
    PathPlannerPath path3;

    try{
      path3= PathPlannerPath.fromPathFile("TopStartToShootT");
    } catch(Exception e){
      e.printStackTrace();
      return Commands.none();
    }

    return AutoBuilder.followPath(path3);
  }



 public Command goMidToShooterPs(){
    PathPlannerPath path6;

    try{
      path6= PathPlannerPath.fromPathFile("MidStartToShootM");
    } catch(Exception e){
      e.printStackTrace();
      return Commands.none();
    }

    return AutoBuilder.followPath(path6);
  }

//climbing only
  public Command goMidtoTower(){
    PathPlannerPath path2;

    try{
      path2= PathPlannerPath.fromPathFile("MidtoTower");
    } catch(Exception e){
      e.printStackTrace();
      return Commands.none();
    }

    return AutoBuilder.followPath(path2);
  }


  public Command goBottomShootertoDepot(){
    PathPlannerPath path4;

    try{
      path4= PathPlannerPath.fromPathFile("ShootMtoDepot");
    } catch(Exception e){
      e.printStackTrace();
      return Commands.none();
    }

    return AutoBuilder.followPath(path4);
  }
  

  public Command goMidShootertoDepot(){
    PathPlannerPath path4;

    try{
      path4= PathPlannerPath.fromPathFile("ShootBtoDepot");
    } catch(Exception e){
      e.printStackTrace();
      return Commands.none();
    }

    return AutoBuilder.followPath(path4);
  }


 public Command goTopShootertoDepot(){
    PathPlannerPath path4;

    try{
      path4= PathPlannerPath.fromPathFile("ShootTtoDepot");
    } catch(Exception e){
      e.printStackTrace();
      return Commands.none();
    }

    return AutoBuilder.followPath(path4);
  }


  public Command goDepotToMid(){
    PathPlannerPath path5;

    try{
      path5= PathPlannerPath.fromPathFile("DepotToMid");
    } catch(Exception e){
      e.printStackTrace();
      return Commands.none();
    }

    return AutoBuilder.followPath(path5);
  }
  
 

  public Command goMidTower(){
    PathPlannerPath path7;

    try{
      path7= PathPlannerPath.fromPathFile("MidtoTower");
    } catch(Exception e){
      e.printStackTrace();
      return Commands.none();
    }

    return AutoBuilder.followPath(path7);
  }
 

  public Command goBottomTower(){
    PathPlannerPath BtTower;

    try{
      BtTower= PathPlannerPath.fromPathFile("BottomToTower");
    } catch(Exception e){
      e.printStackTrace();
      return Commands.none();
    }

    return AutoBuilder.followPath(BtTower);
  }


   public Command goTopTower(){
    PathPlannerPath TpTower;

    try{
      TpTower= PathPlannerPath.fromPathFile("TopToTower");
    } catch(Exception e){
      e.printStackTrace();
      return Commands.none();
    }

    return AutoBuilder.followPath(TpTower);
  }

//delete this soon 
  public Command goRandomPath(){
    PathPlannerPath pathRandom;

    try{
      pathRandom= PathPlannerPath.fromPathFile("randomPath");
    } catch(Exception e){
      e.printStackTrace();
      return Commands.none();
    }

    return AutoBuilder.followPath(pathRandom);
  }
//AUTOROUTINES 

  public Command bottomStartToShootOnly() {
    return Commands.sequence(
            new InstantCommand(() -> System.out.println("Moving from bottom position to Bottom shooting position")),
            goBottomToShootPs(),
            new InstantCommand(() -> System.out.println("Reached bottom shooting position")),
            new InstantCommand(() -> System.out.println("We will now begin shooting")),
            shoot(),
            stop(),
            new InstantCommand(() -> System.out.println("routine complete")),
    );
  } 

  public Command topStartToShootOnly() {
    return Commands.sequence(
            new InstantCommand(() -> System.out.println("Moving from top position to shooting position")),
            goToptoShooterPs(),
            new InstantCommand(() -> System.out.println("Reached top shooting position")),
            new InstantCommand(() -> System.out.println("We will now begin shooting")),
            shoot(),
            stop(),
            new InstantCommand(() -> System.out.println("routine complete")),
    );
  } 

  public Command midStartToShootOnly() {
    return Commands.sequence(
            new InstantCommand(() -> System.out.println("Moving from mid position to shooting position")),
            goMidToShooterPs(),
            new InstantCommand(() -> System.out.println("Reached mid shooting position")),
            new InstantCommand(() -> System.out.println("We will now begin shooting")),
            shoot(),
            stop(),
            new InstantCommand(() -> System.out.println("routine complete")),
    );
  } 

  public Command StartBottomToTower(){
    return Commands.sequence(
            new InstantCommand(()-> System.out.println("We will just be climbing")),
            goBottomTower(),
            climb(),
            stop(),
            new InstantCommand(()-> System.out.println("bottom to climb routine complete"))
    );
  }

   public Command StartTopToTower(){
    return Commands.sequence(
            new InstantCommand(()-> System.out.println("We will just be climbing")),
            goTopTower(),
            climb(),
            stop(),
            new InstantCommand(()-> System.out.println("top to climb routine complete"))
    );
  }

 public Command StartMidToTower(){
    return Commands.sequence(
            new InstantCommand(()-> System.out.println("We will just be climbing")),
            goMidTower(),
            climb(),
            stop(),
            new InstantCommand(()-> System.out.println("mid to climb routine complete"))
    );
  }

  public Command StartBottomShootIntakeEndL1(){
    return Commands.sequence(
            new InstantCommand(()-> System.out.println("Routine 1 starting - ")),
            goBottomToShootPs(),
            shoot(),
            goBottomShootertoDepot(),
            new WaitCommand(3),
            goDepotToMid(),
            goMidTower(),
            climb(),
            stop(),
            new InstantCommand(()-> System.out.println("routine 1 complete"))
    );
  }

  public Command StartMidShootIntakeTower(){
    return Commands.sequence(
            new InstantCommand(()->System.out.println("Moving to Shooter position")),
            goMidToShooterPs(),
            shoot(),
            goMidShootertoDepot(),
            new WaitCommand(3),
            goDepotToMid(),
            goMidTower(),
            climb(),
            stop(),
            new InstantCommand(()->System.out.println("complete routine"))
            );
  }

  public Command StartTopShootIntakeTower(){
    return Commands.sequence(
            new InstantCommand(()-> System.out.println("Moving from top to mid shooting position")),
            goToptoShooterPs(),
            shoot(),
            goTopShootertoDepot(),
            new WaitCommand(3),
            goDepotToMid(),
            goMidTower(),
            climb(),
            stop(),
             new InstantCommand(()->System.out.println("complete routine"))
    );
  }

  public Command MoveRandomCircle(){
    return Commands.sequence(
            new InstantCommand(()->System.out.println("We will be moving in a random circle now!")),
            goRandomPath(),
            new InstantCommand(()->System.out.println("We are done moving in a random circle"))


    );
  }
}