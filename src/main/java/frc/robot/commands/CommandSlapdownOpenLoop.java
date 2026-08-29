package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.slapdown.Slapdown;

public class CommandSlapdownOpenLoop extends Command {
  private final Slapdown slapdown;
  private final double dutyCycle;

  public CommandSlapdownOpenLoop(Slapdown slapdown, double dutyCycle) {
    addRequirements(slapdown);
    this.slapdown = slapdown;
    this.dutyCycle = dutyCycle;
  }

  @Override
  public void initialize() {
    slapdown.setDutyCycle(dutyCycle);
  }

  @Override
  public void execute() {
    slapdown.setDutyCycle(dutyCycle);
  }

  @Override
  public void end(boolean interrupted) {
    slapdown.setDutyCycle(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}