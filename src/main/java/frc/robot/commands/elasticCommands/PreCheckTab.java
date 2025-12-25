package frc.robot.commands.elasticCommands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import frc.robot.util.elastic.Elastic;
import frc.robot.util.limelight.Limelights;

import java.util.function.Supplier;

public class PreCheckTab extends Command {
  private final Supplier<Boolean> mainControllerConnected;
  private final Supplier<Boolean> buttonBoxControllerOneConnected;
  private final Supplier<Boolean> buttonBoxControllerTwoConnected;
  private final Elastic.Tabs tab;

  public PreCheckTab(
      Supplier<Boolean> mainControllerConnected,
      Supplier<Boolean> buttonBoxControllerOneConnected,
      Supplier<Boolean> buttonBoxControllerTwoConnected
  ) {
    this.mainControllerConnected = mainControllerConnected;
    this.buttonBoxControllerOneConnected = buttonBoxControllerOneConnected;
    this.buttonBoxControllerTwoConnected = buttonBoxControllerTwoConnected;

    this.tab = Elastic.Tabs.PRE_CHECK;
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }

  @Override
  public void execute() {
    boolean mainControllerConnected = this.mainControllerConnected.get();
    boolean operatorController1Connected = buttonBoxControllerOneConnected.get();
    boolean operatorController2Connected = buttonBoxControllerTwoConnected.get();

    boolean allControllersConnected =
        mainControllerConnected && operatorController1Connected && operatorController2Connected;
    boolean allSystemsGo = allControllersConnected;

    tab.getEntry("All Controllers").setBoolean(allControllersConnected);
    tab.getEntry("Main Controller").setBoolean(mainControllerConnected);
    tab.getEntry("Op Controller 1").setBoolean(operatorController1Connected);
    tab.getEntry("Op Controller 2").setBoolean(operatorController2Connected);

    allSystemsGo = isLimelightConnected(Limelights.LEFT) && allSystemsGo;
    allSystemsGo = isLimelightConnected(Limelights.RIGHT) && allSystemsGo;
    allSystemsGo = isLimelightConnected(Limelights.REAR) && allSystemsGo;

    tab.getEntry("All Systems Go!").setBoolean(allSystemsGo);
  }

  private boolean isLimelightConnected(Limelights limelight) {
    boolean limelightFound = limelight.isConnected();

    tab.getEntry(limelight.name).setBoolean(limelightFound);

    return limelightFound;
  }

  @Override
  public boolean isFinished() {
    return DriverStation.isEnabled();
  }
}
