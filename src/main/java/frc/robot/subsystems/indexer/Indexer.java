package frc.robot.subsystems.indexer;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.indexer.IndexerIO.IndexerIOInputs;
import org.littletonrobotics.junction.Logger;

public class Indexer extends SubsystemBase {
  private final IndexerIO io;
  private final IndexerIOInputs inputs = new IndexerIOInputs();

  public Indexer(IndexerIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    // Update inputs from IO
    io.updateInputs(inputs);
    Logger.recordOutput("IndexerTemp", inputs.temperatureC);
    Logger.recordOutput("IndexerCurrent", inputs.currentAmps);
    Logger.recordOutput("IndexerVelocity", inputs.velocityRPM);
  }

  /** Open loop duty cycle control. Positive feeds toward the shooter. */
  public void setDutyCycle(double dutyCycle) {
    io.setDutyCycle(dutyCycle);
  }

  public void setBrakeMode(boolean enable) {
    io.setBrakeMode(enable);
  }

  public double getVelocityRPM() {
    return inputs.velocityRPM;
  }
}
