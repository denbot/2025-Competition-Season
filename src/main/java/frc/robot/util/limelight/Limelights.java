package frc.robot.util.limelight;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotController;

import java.io.*;
import java.net.*;
import java.nio.charset.StandardCharsets;
import java.util.HashMap;
import java.util.Map;

public enum Limelights {
  LEFT(
      "limelight-left",
      "10.95.86.11"
  ),
  RIGHT(
      "limelight-right",
      "10.95.86.13"
  ),
  REAR(
      "limelight-rear",
      "10.95.86.12"
  );

  private static final Map<String, Boolean> limelightCache = new HashMap<>();
  private static final Map<String, Long> limelightLastCheckTimer = new HashMap<>();

  public final String name;
  private final String ip;
  private boolean pipelinesUploaded = false;
  private final Map<Integer, File> pipelines;


  Limelights(String name, String ip) {
    this.name = name;
    this.ip = ip;

    pipelines = new HashMap<>();

    File limelightDirectory = new File(Filesystem.getDeployDirectory(),"limelights" + File.separator + name);
    for (int pipelineNum = 0; pipelineNum < 10; pipelineNum++) {
      File pipelineFile = new File(limelightDirectory, String.format("pipeline-%d.json", pipelineNum));
      if(pipelineFile.exists()) {
        pipelines.put(pipelineNum, pipelineFile);
      }
    }
  }

  public void setPipeline(LimelightPipeline pipeline) {
    LimelightHelpers.setPipelineIndex(name, pipeline.pipeline);
  }

  public boolean isConnected() {
    // We only want to check every so often, instead of every 20ms. Every second is sufficient.
    long fpgaTime = RobotController.getFPGATime();
    if (limelightLastCheckTimer.containsKey(name)) {
      long elapsedTime = fpgaTime - limelightLastCheckTimer.get(name);

      if (elapsedTime < 1_000_000) { // 1 second in microseconds
        return limelightCache.getOrDefault(name, false);
      }
      limelightLastCheckTimer.put(name, fpgaTime);
    } else {
      // Okay, so we haven't been checked ever. Let's see if anyone else got checked this loop
      // Note that "this loop" is approximate, we just verify it wasn't in the last 20 ms.
      for (Map.Entry<String, Long> lastChecked : limelightLastCheckTimer.entrySet()) {
        long elapsedTime = fpgaTime - lastChecked.getValue();
        if (elapsedTime < 20) {
          return false; // We haven't been checked and another camera got checked this loop
        }
      }
    }

    boolean limelightFound = isLimelightFound();

    limelightCache.put(name, limelightFound);
    limelightLastCheckTimer.put(name, fpgaTime);

    if(limelightFound && ! pipelinesUploaded) {
      uploadPipelines();
    }

    return limelightFound;
  }

  private boolean isLimelightFound() {
    String url = String.format("http://%s:5807/", ip);

    try {
      HttpURLConnection connection = (HttpURLConnection) new URL(url).openConnection();
      connection.setConnectTimeout(5);
      connection.setReadTimeout(5);
      connection.setRequestMethod("HEAD");
      int responseCode = connection.getResponseCode();

      return responseCode == 200;
    } catch (SocketTimeoutException e) {
      return false;
    } catch (IOException e) {
      throw new RuntimeException(e);
    }
  }

  private void uploadPipelines() {
    for (Map.Entry<Integer, File> entry : pipelines.entrySet()) {
      boolean wasSuccessful = uploadPipeline(entry.getKey(), entry.getValue());

      if(! wasSuccessful) {
        return;
      }
    }

    pipelinesUploaded = true;
    // TODO Expose this status for pre-check
  }

  private boolean uploadPipeline(int pipelineNumber, File file) {
    // TODO Actually use the pipeline number instead of letting it be hard coded into the pipeline file
    try {
      StringBuilder content = new StringBuilder();
      try (BufferedReader reader = new BufferedReader(new FileReader(file))) {
        String line;
        while ((line = reader.readLine()) != null) {
          content.append(line).append(System.lineSeparator());
        }
      }
      String jsonData = content.toString();

      String url = String.format("http://%s:5807/upload-pipeline", ip);
      HttpURLConnection connection = (HttpURLConnection) new URL(url).openConnection();

      connection.setRequestMethod("POST");
      connection.setRequestProperty("Content-Type", "application/json; charset=UTF-8");
      connection.setDoOutput(true);

      try (OutputStream outputStream = connection.getOutputStream()) {
        byte[] input = jsonData.getBytes(StandardCharsets.UTF_8);
        outputStream.write(input, 0, input.length);
      }

      int responseCode = connection.getResponseCode();
      return responseCode == HttpURLConnection.HTTP_OK;
    } catch (IOException e) {
      throw new RuntimeException(e);
    }
  }
}
