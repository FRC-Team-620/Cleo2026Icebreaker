package frc.robot.subsystems.indexer;

import org.littletonrobotics.junction.AutoLog;

public interface IndexerIO {
  @AutoLog
  public static class IndexerIOInputs {
    /** Stator current summed across all four indexer motors. */
    public double currentAmps;

    /** Speed of the lead motor. */
    public double velocityRPM;

    /** Temperature of the hottest of the four motors. */
    public double temperatureC;
  }

  public default void updateInputs(IndexerIOInputs inputs) {}

  /** Open loop duty cycle control. Positive feeds toward the shooter. */
  public default void setDutyCycle(double dutyCycle) {}

  public default void setBrakeMode(boolean enable) {}
}
