package frc.robot.util;

import java.util.function.Function;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.Timestamp;

import frc.robot.util.LoggedTalon.TelemetryLevel;

public class LeveledSignal<T> {
  private final StatusSignal<T> signal;
  public final TelemetryLevel level;
  private Function<T, String> logString;

  public LeveledSignal(StatusSignal<T> signal, TelemetryLevel level, Function<T, String> logString) {
    this.signal = signal;
    this.level = level;
    this.logString = logString;
  }

  public T getValue() {
    return signal.getValue();
  }

  public Timestamp getTimestamp() {
    return signal.getTimestamp();
  }

  public StatusSignal<T> refresh() {
    return signal.refresh();
  }

  public StatusCode getStatus() {
    return signal.getStatus();
  }

  public BaseStatusSignal getBaseSignal() {
    return signal;
  }

  public String getLogString() {
    return logString.apply(getValue());
  }
}
