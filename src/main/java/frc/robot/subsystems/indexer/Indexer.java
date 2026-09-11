package frc.robot.subsystems.indexer;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Indexer extends SubsystemBase {
  private final IndexerIO io;

  public Indexer(IndexerIO io) {
    this.io = io;
  }

  public void setSpeed(double speed) {
    io.setSpeed(speed);
  }

  public void setBrakeMode(boolean enable) {
    io.setBrakeMode(enable);
  }
}
