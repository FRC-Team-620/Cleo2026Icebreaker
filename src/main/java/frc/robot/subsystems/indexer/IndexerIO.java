package frc.robot.subsystems.indexer;

public interface IndexerIO {
  public static class IndexerIOInputs {
    public double motorVoltage;
    public double motorCurrentAMPS;
    public double motorTemperatureC;
    public double motorVelocityRPM;
  }

  public default void updateInputs(IndexerIOInputs inputs) {}

  public default void setSpeed(double speed) {}

  public default void setBrakeMode(boolean enable) {}
}
