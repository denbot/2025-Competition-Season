package frc.robot.commands.rumbleCommands.builders;

import java.util.Optional;

public record RumbleScale(
    ScaleType scaleType,
    Optional<Double> leftTarget,
    Optional<Double> rightTarget,
    double time) {
  public enum ScaleType {
    INSTANT,

    /**
     * Linear function is modeled as f(t) = k * t - [start] where k is our scaleFactor. To solve for
     * k, it's just k = ([target] + [start]) / T. T is how long we want the scale to take.
     */
    LINEAR,

    /**
     * Exponential function is modeled as f(t) = k^(t + [start]) - 1 where k is our scaleFactor. The
     * benefit to this model is we can calculate the scale facture by setting the equation equal to
     * our [target] and solving for how long we want the scale to last. It's the nth root of
     * ([target] + 1) where n is (T + [start]). T is how long we want the scale to take.
     */
    EXPONENTIAL
  }
}
