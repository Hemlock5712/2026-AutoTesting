package frc.robot.utils;

import java.util.HashMap;
import java.util.Map;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public final class LoopProfiler {
  private static final boolean ENABLED = true;
  private static final int REPORT_EVERY_N_CALLS = 10;
  private static final Map<String, Stats> stats = new HashMap<>();

  private LoopProfiler() {}

  public static void measure(String key, Runnable runnable) {
    if (!ENABLED) {
      runnable.run();
      return;
    }

    long start = System.nanoTime();
    try {
      runnable.run();
    } finally {
      record(key, start);
    }
  }

  public static <T> T measure(String key, Supplier<T> supplier) {
    if (!ENABLED) {
      return supplier.get();
    }

    long start = System.nanoTime();
    try {
      return supplier.get();
    } finally {
      record(key, start);
    }
  }

  private static void record(String key, long startNanos) {
    Stats stat = stats.computeIfAbsent(key, Stats::new);
    double elapsedMs = (System.nanoTime() - startNanos) / 1e6;

    stat.calls++;
    stat.latestMs = elapsedMs;
    stat.maxMs = Math.max(stat.maxMs, elapsedMs);

    if (stat.calls % REPORT_EVERY_N_CALLS == 0) {
      Logger.recordOutput(stat.msKey, stat.latestMs);
      Logger.recordOutput(stat.maxMsKey, stat.maxMs);
      stat.maxMs = 0.0;
    }
  }

  private static final class Stats {
    int calls = 0;
    double latestMs = 0.0;
    double maxMs = 0.0;
    final String msKey;
    final String maxMsKey;

    Stats(String key) {
      msKey = "LoopProfiler/" + key + "Ms";
      maxMsKey = "LoopProfiler/" + key + "MaxMs";
    }
  }
}
