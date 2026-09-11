package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.Shooter;

public class CommandShooter extends Command {
  private final Shooter shooter;
  private final double rpm;

  public CommandShooter(Shooter shooter, double rpm) {
    addRequirements(shooter);
    this.shooter = shooter;
    this.rpm = rpm;
  }

  @Override
  public void initialize() {
    // shooter.setRPM(rpm);
    shooter.setVoltage(6);
  }

  @Override
  public void execute() {
    // shooter.setRPM(rpm);
    shooter.setVoltage(6);
  }

  @Override
  public void end(boolean interrupted) {
    // shooter.stop();
    shooter.setVoltage(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
