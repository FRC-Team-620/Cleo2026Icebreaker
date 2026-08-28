package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

  public Shooter(ShooterIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    // Update inputs from IO
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);
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

  /** True when the shooter is running closed loop and is up to its commanded speed. */
  public boolean atSetpoint() {
    return inputs.desiredRPM != 0.0
        && Math.abs(inputs.desiredRPM - inputs.velocityRPM) <= ShooterConstants.kShooterTolerance;
  }
}
