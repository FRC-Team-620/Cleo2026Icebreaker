package frc.robot.subsystems.indexer;

public class SimIndexerIO implements IndexerIO {
  private double dutyCycle;

  @Override
  public void updateInputs(IndexerIOInputs inputs) {
    // Simulate the indexer motor behavior in simulation
    inputs.currentAmps = 5.0; // Simulated current
    inputs.temperatureC = 30.0; // Simulated temperature
    inputs.velocityRPM = dutyCycle * 6000.0; // Simulated velocity
  }

  @Override
  public void setDutyCycle(double dutyCycle) {
    this.dutyCycle = dutyCycle;
  }

  @Override
  public void setBrakeMode(boolean enable) {
    // Simulate setting the brake mode of the indexer motors in simulation
    System.out.println(
        "Simulated indexer motor brake mode set to: " + (enable ? "enabled" : "disabled"));
  }
}
