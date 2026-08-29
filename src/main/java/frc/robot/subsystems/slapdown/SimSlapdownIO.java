package frc.robot.subsystems.slapdown;

import edu.wpi.first.math.MathUtil;
import frc.robot.Constants;

public class SimSlapdownIO implements SlapdownIO {
  private static final double MAX_VELOCITY_DEGREES_PER_SEC = 360.0;
  private static final double SIM_KP = 8.0;

  private double angleDegrees = 0.0;
  private double goalAngleDegrees = 0.0;
  private double appliedDutyCycle = 0.0;
  private boolean closedLoop = false;

  @Override
  public void updateInputs(SlapdownIOInputs inputs) {
    // Fake closed loop by driving the duty cycle proportional to angle error
    double dutyCycle =
        closedLoop
            ? MathUtil.clamp(SIM_KP * (goalAngleDegrees - angleDegrees) / 360.0, -1.0, 1.0)
            : appliedDutyCycle;
    double velocityDegreesPerSec = dutyCycle * MAX_VELOCITY_DEGREES_PER_SEC;
    angleDegrees += velocityDegreesPerSec * Constants.ksimTimestep;

    inputs.currentAngleDegrees = angleDegrees;
    inputs.velocityRPM = velocityDegreesPerSec / 360.0 * 60.0;
    inputs.currentAmps = Math.abs(dutyCycle) * 40.0; // Simulated current
    inputs.temperatureC = 30.0; // Simulated temperature
    inputs.goalAngleDegrees = closedLoop ? goalAngleDegrees : 0.0;
  }

  @Override
  public void setAngle(double angleDegrees) {
    closedLoop = true;
    goalAngleDegrees = angleDegrees;
  }

  @Override
  public void setDutyCycle(double dutyCycle) {
    closedLoop = false;
    appliedDutyCycle = dutyCycle;
  }

  @Override
  public void setBrakeMode(boolean enable) {
    // Simulate setting the brake mode of the slapdown motor in simulation
    System.out.println(
        "Simulated slapdown motor brake mode set to: " + (enable ? "enabled" : "disabled"));
  }
}
