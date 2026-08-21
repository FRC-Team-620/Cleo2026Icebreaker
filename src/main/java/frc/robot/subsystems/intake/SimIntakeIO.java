package frc.robot.subsystems.intake;

public class SimIntakeIO implements IntakeIO {
  private double dutyCycle;

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    // Simulate the intake motor behavior in simulation
    // For example, you can set the inputs based on some simulated values
    inputs.motorVoltage = 12.0; // Simulated voltage
    inputs.motorCurrentAMPS = 5.0; // Simulated current
    inputs.motorTemperatureC = 30.0; // Simulated temperature
    inputs.motorVelocityRPM = dutyCycle * 10.0; // Simulated velocity
  }

  @Override
  public void setSpeedDutyCycle(double dutyCycle) {
    // Simulate setting the speed of the intake motor in simulation
    this.dutyCycle = dutyCycle;
  }

  @Override
  public void setBrakeMode(boolean enable) {
    // Simulate setting the brake mode of the intake motor in simulation
    System.out.println(
        "Simulated intake motor brake mode set to: " + (enable ? "enabled" : "disabled"));
  }
}
