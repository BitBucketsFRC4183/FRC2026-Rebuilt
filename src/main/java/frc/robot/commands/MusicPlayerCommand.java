package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class MusicPlayerCommand extends Command {
  private Songs song = Songs.CLASH;

  public static enum Songs {
    CLASH("sounds/bootup-cr2.chrp");

    public final String filepath;

    Songs(String filepath) {
      this.filepath = filepath;
    }
  }

  public MusicPlayerCommand(Songs song) {
    this.song = song;
  }

  @Override
  public void initialize() {
    Robot.orchestra.stop();
    Robot.orchestra.loadMusic(song.filepath);
  }

  @Override
  public void execute() {
    Robot.orchestra.play();
  }
}
