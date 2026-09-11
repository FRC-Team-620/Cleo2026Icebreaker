package frc.robot.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;

/**
 * Physics sim implementation of the shooter IO. All four motors drive the same flywheel model,
 * since both sides are commanded to the same speed. Simulation is always based on voltage control.
 */
public class SimShooterIO implements ShooterIO {
  private static final DCMotor GEARBOX = DCMotor.getKrakenX60Foc(4);
  // Rough flywheel moment of inertia, tune to match how fast the real shooter spins up
  private static final double MOI_KG_METERS_SQUARED = 0.004;
  private static final double SIM_KP = 0.05;
  private static final double SIM_KV = 1.0 / GEARBOX.KvRadPerSecPerVolt;

  private final DCMotorSim flywheelSim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(
              GEARBOX, MOI_KG_METERS_SQUARED, ShooterConstants.kGearRatio),
          GEARBOX);
  private final PIDController controller = new PIDController(SIM_KP, 0.0, 0.0);

  private boolean closedLoop = false;
  private double desiredRPM = 0.0;
  private double appliedVolts = 0.0;

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    // Run closed loop control
    if (closedLoop) {
      double setpointRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(desiredRPM);
      appliedVolts =
          SIM_KV * setpointRadPerSec
              + controller.calculate(flywheelSim.getAngularVelocityRadPerSec(), setpointRadPerSec);
    } else {
      controller.reset();
    }

    // Update the simulation state
    flywheelSim.setInputVoltage(MathUtil.clamp(appliedVolts, -12.0, 12.0));
    flywheelSim.update(Constants.ksimTimestep);

    inputs.currentAmps = Math.abs(flywheelSim.getCurrentDrawAmps());
    inputs.velocityRPM =
        Units.radiansPerSecondToRotationsPerMinute(flywheelSim.getAngularVelocityRadPerSec());
    inputs.temperatureC = 25.0; // Simulated temperature
    inputs.desiredRPM = closedLoop ? desiredRPM : 0.0;
  }

  @Override
  public void setRPM(double rpm) {
    closedLoop = true;
    desiredRPM = rpm;
  }

  @Override
  public void setVoltage(double volts) {
    closedLoop = false;
    desiredRPM = 0.0;
    appliedVolts = volts;
  }

  @Override
  public void setBrakeMode(boolean enable) {
    // Simulate setting the brake mode of the shooter motors in simulation
    System.out.println(
        "Simulated shooter motor brake mode set to: " + (enable ? "enabled" : "disabled"));
  }
}
