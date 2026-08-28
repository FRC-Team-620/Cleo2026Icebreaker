package frc.robot.subsystems.intake;

import static frc.robot.util.PhoenixUtil.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;

public class KrakenIntakeIO implements IntakeIO {
  private TalonFX intakeLeftMotor;
  private TalonFX intakeRightMotor;

  private TalonFXConfiguration leadConfig;
  private TalonFXConfiguration followConfig;

  // status signals
  private final StatusSignal<Voltage> leadAppliedVolts;
  private final StatusSignal<Current> leadCurrent;
  private final StatusSignal<Temperature> leadTemperature;
  private final StatusSignal<AngularVelocity> leadVelocity;

  private final StatusSignal<Voltage> followAppliedVolts;
  private final StatusSignal<Current> followCurrent;
  private final StatusSignal<Temperature> followTemperature;
  private final StatusSignal<AngularVelocity> followVelocity;

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

    // Create status signals for the lead motor
    leadAppliedVolts = intakeLeftMotor.getMotorVoltage();
    leadCurrent = intakeLeftMotor.getStatorCurrent();
    leadTemperature = intakeLeftMotor.getDeviceTemp();
    leadVelocity = intakeLeftMotor.getVelocity();

    // Create status signals for the follower motor
    followAppliedVolts = intakeRightMotor.getMotorVoltage();
    followCurrent = intakeRightMotor.getStatorCurrent();
    followTemperature = intakeRightMotor.getDeviceTemp();
    followVelocity = intakeRightMotor.getVelocity();

    // Configure periodic frames
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        leadAppliedVolts,
        leadCurrent,
        leadTemperature,
        leadVelocity,
        followAppliedVolts,
        followCurrent,
        followTemperature,
        followVelocity);
    ParentDevice.optimizeBusUtilizationForAll(intakeLeftMotor, intakeRightMotor);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    // Refresh all signals
    BaseStatusSignal.refreshAll(
        leadAppliedVolts,
        leadCurrent,
        leadTemperature,
        leadVelocity,
        followAppliedVolts,
        followCurrent,
        followTemperature,
        followVelocity);

    // Voltage and current are summed across both motors, temperature is the hottest of the two,
    // and velocity is taken from the lead motor only (the follower mirrors it).
    inputs.motorVoltage =
        leadAppliedVolts.getValueAsDouble();
    inputs.motorCurrentAMPS = leadCurrent.getValueAsDouble() + followCurrent.getValueAsDouble();
    inputs.motorTemperatureC =
        Math.max(leadTemperature.getValueAsDouble(), followTemperature.getValueAsDouble());
    inputs.motorVelocityRPM = leadVelocity.getValueAsDouble() * 60.0;
  }

  @Override
  public void setSpeedDutyCycle(double dutyCycle) {
    // Set the speed of the Kraken intake motor using the specified duty cycle
    intakeLeftMotor.set(dutyCycle);
  }

  @Override
  public void setBrakeMode(boolean enable) {
    // Enable or disable brake mode for both Kraken intake motors
    NeutralModeValue neutralMode = enable ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    // (boolean) ? iftrue : ifffalse
    leadConfig.MotorOutput.NeutralMode = neutralMode;
    followConfig.MotorOutput.NeutralMode = neutralMode;

    // Only the motor output portion of the config needs to be re-sent
    tryUntilOk(5, () -> intakeLeftMotor.getConfigurator().apply(leadConfig.MotorOutput, 0.25));
    tryUntilOk(5, () -> intakeRightMotor.getConfigurator().apply(followConfig.MotorOutput, 0.25));
  }
}
