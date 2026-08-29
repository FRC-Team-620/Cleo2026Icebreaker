package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
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
    shooter.setRPM(rpm);
  }

  @Override
  public void execute() {
    shooter.setRPM(Constants.ShooterConstants.kBaseRPM);
  }

  @Override
  public void end(boolean interrupted) {
    shooter.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}