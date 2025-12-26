package frc.robot.commands;

import com.ctre.phoenix6.Orchestra;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.CanBeAnInstrument;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * A command that manages the playback of music through CTRE TalonFX motors using {@link Orchestra}.
 *
 * <p>This command automatically populates a dashboard chooser with all available {@code .chrp} files
 * from the deploy directory. It handles the loading, playing, pausing, and swapping of songs
 * based on dashboard selection and controller input.
 *
 * <p>It uses an internal {@link EventLoop} to handle button bindings that are only active
 * while this command is running, allowing for contextual controls like a play/pause toggle.
 */
public class OrchestraPlayer extends Command {
  private final Orchestra orchestra = new Orchestra();
  private final EventLoop eventLoop;

  private final LoggedDashboardChooser<String> songChooser;
  private String loadedSong = null;

  public OrchestraPlayer(
      CommandXboxController controller,
      CanBeAnInstrument... instruments
  ) {
    this.eventLoop = new EventLoop();

    for (var instrument : instruments) {
      instrument.addInstruments(orchestra);
    }

    // Let's load all of our songs into our song chooser so the user can decide what to play
    songChooser = new LoggedDashboardChooser<>("Song Choice");

    String[] chrpFiles = Filesystem.getDeployDirectory().list((dir, name) -> name.endsWith(".chrp"));
    if (chrpFiles == null) {
      chrpFiles = new String[0];
    }

    for (String fileName : chrpFiles) {
      // We just need the filename for the Orchestra class, but might as well try to make the dropdown look pretty
      String songName = fileName
          .substring(0, fileName.length() - 5)
          .replace('_', ' ')
          .replace('-', ' ');
      songChooser.addOption(songName, fileName);
    }

    // Simple play / pause button that is only registered for this command
    controller.a(eventLoop)
        .onTrue(Commands.runOnce(() -> {
          if (orchestra.isPlaying()) {
            orchestra.pause();
          } else {
            orchestra.play();
          }
        }));
  }

  @Override
  public void execute() {
    this.eventLoop.poll();

    String selectedSong = songChooser.get();
    // The user just selected a song for the first time. Load it and play it.
    if(loadedSong == null && selectedSong != null) {
      orchestra.loadMusic(selectedSong);
      orchestra.play();
      loadedSong = selectedSong;
    }

    // Did the user select a new song in the middle of our current song? Better stop the current song
    if (loadedSong != null && !loadedSong.equals(selectedSong)) {
      boolean isPlaying = orchestra.isPlaying();
      if (isPlaying) {
        orchestra.stop();
      }

      orchestra.loadMusic(selectedSong);
      loadedSong = selectedSong;

      // Immediately jump into the next song if we were playing music before
      if (isPlaying) {
        orchestra.play();
      }
    }
  }
}
