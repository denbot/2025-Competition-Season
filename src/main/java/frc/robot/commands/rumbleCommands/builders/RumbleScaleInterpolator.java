package frc.robot.commands.rumbleCommands.builders;

public class RumbleScaleInterpolator {
  public final double runTime; // How long this should take

  private final RumbleScale.ScaleType scaleType;
  private final double scaleFactor; // What scale factor are we using based on our scale type
  private final double start; // What rumble value did we start with
  private final double target; // What rumble value are we heading to

  public RumbleScaleInterpolator(
      RumbleScale.ScaleType scaleType, double start, double target, double runTime) {
    this.scaleType = scaleType;
    this.start = start;
    this.target = target;
    this.runTime = runTime;

    if (scaleType == RumbleScale.ScaleType.INSTANT) {
      scaleFactor = 1;
      if (Math.abs(target - start) > 0.001) {
        throw new IllegalArgumentException(
            "Start and target must be identical for ScaleType of NONE");
      }
    } else if (scaleType == RumbleScale.ScaleType.EXPONENTIAL) {
      double n = runTime + start;
      scaleFactor = Math.pow(target + 1, 1.0 / n);
    } else if (scaleType == RumbleScale.ScaleType.LINEAR) {
      scaleFactor = (target + start) / runTime;
    } else {
      throw new IllegalArgumentException(String.format("Unknown ScaleType %s", scaleType));
    }
  }

  public static RumbleScaleInterpolator stop() {
    return instantly(0, 0);
  }

  public static RumbleScaleInterpolator instantly(double rumble, double time) {
    return new RumbleScaleInterpolator(
        RumbleScale.ScaleType.INSTANT,
        rumble,
        rumble,
        time
    );
  }

  public double rumbleAtTime(double time) {
    if (time >= this.runTime) {
      return target;
    }

    if (time <= 0) {
      return start;
    }

    if (scaleType == RumbleScale.ScaleType.INSTANT) {
      return start;
    }
    if (scaleType == RumbleScale.ScaleType.LINEAR) {
      return scaleFactor * (time - start);
    } else if (scaleType == RumbleScale.ScaleType.EXPONENTIAL) {
      return Math.pow(scaleFactor, time + start) - 1;
    } else {
      throw new IllegalArgumentException(String.format("Unknown ScaleType %s", scaleType));
    }
  }
}
