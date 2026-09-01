package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.indexer.Indexer;

public class CommandIndexer extends Command {
  private final Indexer indexer;
  private final double dutyCycle;

  public CommandIndexer(Indexer indexer, double dutyCycle) {
    addRequirements(indexer);
    this.indexer = indexer;
    this.dutyCycle = dutyCycle;
  }

  @Override
  public void initialize() {
    indexer.setSpeed(dutyCycle);
  }

  @Override
  public void execute() {
    indexer.setSpeed(dutyCycle);
  }

  @Override
  public void end(boolean interrupted) {
    indexer.setSpeed(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
