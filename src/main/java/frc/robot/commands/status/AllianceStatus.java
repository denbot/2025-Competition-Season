package frc.robot.commands.status;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.led.LEDController;

import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Second;

public class AllianceStatus extends Command {
  private final Command noAlliance;
  private final Command redAlliance;
  private final Command blueAlliance;

  private Command lastCommand = null;


  public AllianceStatus(
      LEDController ledController
  ) {

    noAlliance = ledController.run(
        LEDPattern
            .rainbow(255, 128)
            .scrollAtRelativeSpeed(Percent.per(Second).of(25))
    );
    redAlliance = ledController.run(LEDPattern.solid(Color.kRed).atBrightness(Percent.of(50)));
    blueAlliance = ledController.run(LEDPattern.solid(Color.kBlue).atBrightness(Percent.of(50)));
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }

  @Override
  public void execute() {
    Command toRun = getCommandToRun();

    if (toRun == lastCommand) {
      return;
    }

    if (lastCommand != null && lastCommand.isScheduled()) {
      // It would probably get interrupted anyway if it used overlapping LEDs, but we're not going to risk that
      lastCommand.cancel();
    }

    lastCommand = toRun;
    toRun.schedule();
  }

  private Command getCommandToRun() {
    var allianceOptional = DriverStation.getAlliance();
    if(allianceOptional.isEmpty()) {
      return noAlliance;
    }

    if(allianceOptional.get() == DriverStation.Alliance.Red) {
      return redAlliance;
    } else {
      return blueAlliance;
    }
  }

  @Override
  public boolean isFinished() {
    return DriverStation.isEnabled();
  }
}
