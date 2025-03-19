package frc.robot.util;

import edu.wpi.first.hal.HALUtil;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class Suppliers {
  private static class WithExpiration<T> implements Supplier<T> {
    private final Supplier<T> source;
    private final long millisecondsToRefresh;
    public long lastFPGATime = 0;
    private T cache;

    public WithExpiration(Supplier<T> supplier, long millisecondsToRefresh) {
      this.source = supplier;
      this.millisecondsToRefresh = millisecondsToRefresh;
    }

    @Override
    public T get() {
      long fpgaTime = HALUtil.getFPGATime();
      long elapsedTime = fpgaTime - lastFPGATime;
      if (lastFPGATime != 0 && elapsedTime < millisecondsToRefresh) {
        return cache;
      }

      lastFPGATime = fpgaTime;
      cache = source.get();
      return cache;
    }
  }

  public static <T> Supplier<T> memoizeWithExpiration(
      Supplier<T> supplier, long millisecondsToRefresh) {
    return new WithExpiration<>(supplier, millisecondsToRefresh);
  }

  public static DoubleSupplier memoizeWithExpiration(
      DoubleSupplier supplier, long millisecondsToRefresh) {
    WithExpiration<Double> doubleWithExpiration =
        new WithExpiration<>(supplier::getAsDouble, millisecondsToRefresh);
    return doubleWithExpiration::get;
  }
}
