package frc.robot.commands.elasticCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import frc.robot.util.elastic.Elastic;
import java.io.IOException;
import java.net.HttpURLConnection;
import java.net.SocketTimeoutException;
import java.net.URL;
import java.util.HashMap;
import java.util.Map;

public class PreCheckTab extends Command {
  private final CommandGenericHID mainController;
  private final CommandGenericHID operatorController1;
  private final CommandGenericHID operatorController2;
  private final Elastic.Tabs tab;
  private final Map<String, Boolean> limelightCache;
  private final Map<String, Integer> limelightLastChecked;

  public PreCheckTab(
      CommandGenericHID mainController,
      CommandGenericHID operatorController1,
      CommandGenericHID operatorController2) {
    this.mainController = mainController;
    this.operatorController1 = operatorController1;
    this.operatorController2 = operatorController2;

    this.limelightCache = new HashMap<>();
    this.limelightLastChecked = new HashMap<>();

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

    allSystemsGo = pingIP("limelight-left", "10.95.86.10") && allSystemsGo;
    allSystemsGo = pingIP("limelight-right", "10.95.86.11") && allSystemsGo;
    allSystemsGo = pingIP("limelight-rear", "10.95.86.12") && allSystemsGo;

    tab.getEntry("All Systems Go!").setBoolean(allSystemsGo);
  }

  private boolean pingIP(String name, String ip) {
    // We only want to check every so often, instead of every 20ms. Every second is sufficient.
    if (limelightLastChecked.containsKey(name)) {
      int checkCount = limelightLastChecked.get(name) + 1; // # of loops since last check
      limelightLastChecked.put(name, checkCount);

      if (checkCount < 50) {
        return limelightCache.getOrDefault(name, false);
      }
    } else {
      // Okay, so we haven't been checked ever. Let's see if anyone else got checked this loop
      for (Map.Entry<String, Integer> lastChecked : limelightLastChecked.entrySet()) {
        if (lastChecked.getValue() == 0) {
          return false; // We haven't been checked and another camera got checked this loop
        }
      }
    }

    String url = String.format("http://%s/", ip);

    boolean limelightFound;

    try {
      HttpURLConnection connection = (HttpURLConnection) new URL(url).openConnection();
      connection.setConnectTimeout(5);
      connection.setReadTimeout(5);
      connection.setRequestMethod("HEAD");
      int responseCode = connection.getResponseCode();

      limelightFound = responseCode == 200;
    } catch (SocketTimeoutException e) {
      limelightFound = false;
    } catch (IOException e) {
      throw new RuntimeException(e);
    }

    tab.getEntry(name).setBoolean(limelightFound);
    this.limelightCache.put(name, limelightFound);
    this.limelightLastChecked.put(name, 0);

    return limelightFound;
  }
}
