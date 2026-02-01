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
  public Command goBottomToMid() {
    PathPlannerPath path1;

    try {
      // Load the path from the deploy/pathplanner folder by name
      path1 = PathPlannerPath.fromPathFile("BottomStartToMid");
    } catch (Exception e) {
      e.printStackTrace();
      return Commands.none();
    }

    // Create a path-following command using AutoBuilder
    return AutoBuilder.followPath(path1);
  }

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
   // public Command goMidtoTower(){
   // PathPlannerPath path2 = PathPlannerPath.fromChoreoTrajectory("MidtoTower");

   // return AutoBuilder.followPath(path2);
  //}

  public Command goToptoMid(){
    PathPlannerPath path3;

    try{
      path3= PathPlannerPath.fromPathFile("TopStarttoMid");
    } catch(Exception e){
      e.printStackTrace();
      return Commands.none();
    }

    return AutoBuilder.followPath(path3);
  }
  /*public Command goToptoMid(){
    PathPlannerPath path3 = PathPlannerPath.fromChoreoTrajectory("TopStarttoMid");

    return AutoBuilder.followPath(path3);
  }*/

  public Command goMidtoDepot(){
    PathPlannerPath path4;

    try{
      path4= PathPlannerPath.fromPathFile("MidtoDepot");
    } catch(Exception e){
      e.printStackTrace();
      return Commands.none();
    }

    return AutoBuilder.followPath(path4);
  }
  /*public Command goMidtoDepot(){
    PathPlannerPath path4 = PathPlannerPath.fromChoreoTrajectory("MidtoDepot");

    return AutoBuilder.followPath(path4);
  }*/
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
  /*public Command goDepotToMid(){
    PathPlannerPath path5 = PathPlannerPath.fromChoreoTrajectory("DepotToMid");

    return AutoBuilder.followPath(path5);
  }*/
  public Command goMidToTower(){
    PathPlannerPath path6;

    try{
      path6= PathPlannerPath.fromPathFile("MidStarttoTower");
    } catch(Exception e){
      e.printStackTrace();
      return Commands.none();
    }

    return AutoBuilder.followPath(path6);
  }
  /*public Command goMidToTower(){
    PathPlannerPath path6  = PathPlannerPath.fromChoreoTrajectory("MidStarttoTower");

    return AutoBuilder.followPath(path6);
  }*/
  public Command goMidToMid(){
    PathPlannerPath path7;

    try{
      path7= PathPlannerPath.fromPathFile("MidStarttoTower");
    } catch(Exception e){
      e.printStackTrace();
      return Commands.none();
    }

    return AutoBuilder.followPath(path7);
  }
  /*public Command goMidToMid(){
    PathPlannerPath path7 = PathPlannerPath.fromChoreoTrajectory("MidStarttoMid");
      return null;
  }*/
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

  /*public Command goRandomPath() {
      PathPlannerPath pathRandom  = PathPlannerPath.fromChoreoTrajectory("randomPath");

    return AutoBuilder.followPath(pathRandom);
  }*/

  public Command bottomToMidAuto() {
    return Commands.sequence(
            new InstantCommand(() -> System.out.println("Bottom to Mid starting")),
            goBottomToMid(),
            new InstantCommand(() -> System.out.println("Reached mid position"))
            );
  }
  public Command StartBottomShootEndTower(){
    return Commands.sequence(
            new InstantCommand(()-> System.out.println("Routine 1 starting")),
            goBottomToMid(),
            shoot(),
            goMidtoDepot(),
            new WaitCommand(3),
            goDepotToMid(),
            goMidToTower(),
            climb(),
            stop(),
            new InstantCommand(()-> System.out.println("routine 1 complete"))
    );
  }

  public Command StartMidShootIntakeTower(){
    return Commands.sequence(
            new InstantCommand(()->System.out.println("Moving to Shooter position")),
            goMidToMid(),
            shoot(),
            goMidtoDepot(),
            new WaitCommand(3),
            goDepotToMid(),
            goMidToTower(),
            climb(),
            stop(),
            new InstantCommand(()->System.out.println("complete routine"))
            );
  }

  public Command StartTopShootIntakeTower(){
    return Commands.sequence(
            new InstantCommand(()-> System.out.println("Moving from top to mid shooting position")),
            goToptoMid(),
            shoot(),
            goMidtoDepot(),
            new WaitCommand(3),
            goDepotToMid(),
            goMidToTower(),
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