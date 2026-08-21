package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;

public class CommandIntake extends Command {
  private final Intake intake;
  private final double dutyCycle;

  public CommandIntake(Intake intake, double dutyCycle) {
    addRequirements(intake);
    this.intake = intake;
    this.dutyCycle = dutyCycle;
  }

  @Override
  public void initialize() {
    intake.setSpeedDutyCycle(dutyCycle);
  }

  @Override
  public void execute() {
    intake.setSpeedDutyCycle(dutyCycle);
  }

  @Override
  public void end(boolean interrupted) {
    intake.setSpeedDutyCycle(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
