package frc.robot.commands.elasticCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import frc.robot.util.elastic.Elastic;
import frc.robot.util.limelight.Limelights;

public class PreCheckTab extends Command {
  private final CommandGenericHID mainController;
  private final CommandGenericHID operatorController1;
  private final CommandGenericHID operatorController2;
  private final Elastic.Tabs tab;

  public PreCheckTab(
      CommandGenericHID mainController,
      CommandGenericHID operatorController1,
      CommandGenericHID operatorController2) {
    this.mainController = mainController;
    this.operatorController1 = operatorController1;
    this.operatorController2 = operatorController2;

    this.tab = Elastic.Tabs.PRE_CHECK;
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }

  @Override
  public void execute() {
    boolean mainControllerConnected = mainController.isConnected();
    boolean operatorController1Connected = operatorController1.isConnected();
    boolean operatorController2Connected = operatorController2.isConnected();

    boolean allControllersConnected =
        mainControllerConnected && operatorController1Connected && operatorController2Connected;
    boolean allSystemsGo = allControllersConnected;

    tab.getEntry("All Controllers").setBoolean(allControllersConnected);
    tab.getEntry("Main Controller").setBoolean(mainControllerConnected);
    tab.getEntry("Op Controller 1").setBoolean(operatorController1Connected);
    tab.getEntry("Op Controller 2").setBoolean(operatorController2Connected);

    allSystemsGo = isLimelightConnected(Limelights.LEFT) && allSystemsGo;
    allSystemsGo = isLimelightConnected(Limelights.RIGHT) && allSystemsGo;
    // allSystemsGo = isLimelightConnected(Limelights.REAR) && allSystemsGo;

    tab.getEntry("All Systems Go!").setBoolean(allSystemsGo);
  }

  private boolean isLimelightConnected(Limelights limelight) {
    boolean limelightFound = limelight.isConnected();

    tab.getEntry(limelight.name).setBoolean(limelightFound);

    return limelightFound;
  }
}
