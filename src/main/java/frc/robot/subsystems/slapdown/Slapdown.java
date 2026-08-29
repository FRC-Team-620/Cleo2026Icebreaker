package frc.robot.subsystems.slapdown;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.slapdown.SlapdownIO.SlapdownIOInputs;
import org.littletonrobotics.junction.Logger;

public class Slapdown extends SubsystemBase {
  private final SlapdownIO io;
  private final SlapdownIOInputs inputs = new SlapdownIOInputs();

  public Slapdown(SlapdownIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    // Update inputs from IO
    io.updateInputs(inputs);
    Logger.recordOutput("SlapdownAngle", inputs.currentAngleDegrees);
    Logger.recordOutput("SlapdownVelocity", inputs.velocityRPM);
    Logger.recordOutput("SlapdownCurrent", inputs.currentAmps);
    Logger.recordOutput("SlapdownTemp", inputs.temperatureC);
    Logger.recordOutput("SlapdownGoalAngle", inputs.goalAngleDegrees);
    Logger.recordOutput("SlapdownInOpenLoop", inputs.inCloseLoop);
  }

  /** Closed loop position control. */
  public void setAngle(double angleDegrees) {
    io.setAngle(angleDegrees);
  }

  /** Open loop duty cycle control. */
  public void setDutyCycle(double dutyCycle) {
    io.setDutyCycle(dutyCycle);
  }

  public void setBrakeMode(boolean enable) {
    io.setBrakeMode(enable);
  }

  public double getAngleDegrees() {
    return inputs.currentAngleDegrees;
  }
}
