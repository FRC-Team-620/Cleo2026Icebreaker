package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.shooter.ShooterIO.ShooterIOInputs;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
  private final ShooterIO io;
  private final ShooterIOInputs inputs = new ShooterIOInputs();

  public Shooter(ShooterIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    // Update inputs from IO
    io.updateInputs(inputs);
    // Logger.processInputs("Shooter", inputs);
    Logger.recordOutput("IntakeTemp", inputs.temperatureC);
    Logger.recordOutput("IntakeCurrent", inputs.currentAmps);
    Logger.recordOutput("IntakeVelocity", inputs.velocityRPM);
  }

  /** Closed loop velocity control. Positive spins the flywheels in the shooting direction. */
  public void setRPM(double rpm) {
    io.setRPM(rpm);
  }

  /** Open loop voltage control. Positive spins the flywheels in the shooting direction. */
  public void setVoltage(double volts) {
    io.setVoltage(volts);
  }

  public void setBrakeMode(boolean enable) {
    io.setBrakeMode(enable);
  }

  public double getRPM() {
    return inputs.velocityRPM;
  }

  public void stop() {
    io.stop();
  }

  /** True when the shooter is running closed loop and is up to its commanded speed. */
  public boolean atSetpoint() {
    return inputs.desiredRPM != 0.0
        && Math.abs(inputs.desiredRPM - inputs.velocityRPM) <= ShooterConstants.kShooterTolerance;
  }
}
