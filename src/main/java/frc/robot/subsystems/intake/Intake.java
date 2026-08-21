package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.intake.IntakeIO.IntakeIOInputs;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  private final IntakeIO io;
  private final IntakeIOInputs inputs = new IntakeIOInputs();

  public Intake(IntakeIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    // Update inputs from IO
    io.updateInputs(inputs);
    Logger.recordOutput("IntakeTemp", inputs.motorTemperatureC);
    Logger.recordOutput("IntakeCurrent", inputs.motorCurrentAMPS);
    Logger.recordOutput("IntakeVelocity", inputs.motorVelocityRPM);

    if (inputs.motorTemperatureC >= 50.0) {
      setSpeedDutyCycle(0);
    }
  }

  public void setSpeedDutyCycle(double dutyCycle) {
    io.setSpeedDutyCycle(dutyCycle);
  }

  public void setBrakeMode(boolean enable) {
    io.setBrakeMode(enable);
  }
}
