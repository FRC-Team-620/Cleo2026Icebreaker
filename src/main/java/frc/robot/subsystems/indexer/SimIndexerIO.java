package frc.robot.subsystems.indexer;

public class SimIndexerIO implements IndexerIO {
  private double dutyCycle;

  @Override
  public void updateInputs(IndexerIOInputs inputs) {
    /*public double motorVoltage;
    public double motorCurrentAMPS;
    public double motorTemperatureC;
    public double motorVelocityRPM; */
    // Simulate the indexer motor behavior in simulation
    inputs.motorCurrentAMPS = 5.0; // Simulated current
    inputs.motorTemperatureC = 30.0; // Simulated temperature
    inputs.motorVelocityRPM = dutyCycle * 6000.0; // Simulated velocity
  }

  @Override
  public void setSpeed(double dutyCycle) {
    this.dutyCycle = dutyCycle;
  }

  @Override
  public void setBrakeMode(boolean enable) {
    // Simulate setting the brake mode of the indexer motors in simulation
    System.out.println(
        "Simulated indexer motor brake mode set to: " + (enable ? "enabled" : "disabled"));
  }
}
