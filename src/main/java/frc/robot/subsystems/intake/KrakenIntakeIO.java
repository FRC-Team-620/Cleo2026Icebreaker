package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.Constants;

public class KrakenIntakeIO implements IntakeIO {
  private TalonFX intakeLeftMotor;
  private TalonFX intakeRightMotor;

  private TalonFXConfiguration leadConfig;
  private TalonFXConfiguration followConfig;

  // status signals

  public KrakenIntakeIO() {
    // Initialize the TalonFX motor controllers for the Kraken intake
    intakeLeftMotor = new TalonFX(Constants.CAN.kIntakeMotorID); // Replace with actual CAN ID
    intakeRightMotor =
        new TalonFX(Constants.CAN.kIntakeFollowerMotorID); // Replace with actual CAN ID

    // Configure the lead and follow motor controllers
    leadConfig = new TalonFXConfiguration();
    leadConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    leadConfig.CurrentLimits.StatorCurrentLimit = 40;
    leadConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    // Set up the lead motor (intakeLeftMotor) configuration
    // For example, set PID gains, current limits, etc.
    // leadConfig.slot0.kP = 0.1;
    // leadConfig.slot0.kI = 0.0;
    // leadConfig.slot0.kD = 0.0;
    // leadConfig.slot0.kF = 0.0;

    intakeLeftMotor.getConfigurator().apply(leadConfig);

    followConfig = new TalonFXConfiguration();
    followConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    followConfig.CurrentLimits.StatorCurrentLimit = 40;
    followConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    intakeRightMotor.getConfigurator().apply(followConfig);
    intakeRightMotor.setControl(
        new Follower(Constants.CAN.kIntakeMotorID, MotorAlignmentValue.Opposed));
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    // Update the inputs from the Kraken intake hardware
    // For example, read motor voltage, current, temperature, and velocity
    // and set them in the inputs object.
  }

  @Override
  public void setSpeedDutyCycle(double dutyCycle) {
    // Set the speed of the Kraken intake motor using the specified duty cycle
    intakeLeftMotor.set(dutyCycle);
  }

  @Override
  public void setBrakeMode(boolean enable) {
    // Enable or disable brake mode for the Kraken intake motor
  }
}
