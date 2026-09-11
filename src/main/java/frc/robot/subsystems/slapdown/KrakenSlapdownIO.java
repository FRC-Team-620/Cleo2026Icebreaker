package frc.robot.subsystems.slapdown;

import static frc.robot.util.PhoenixUtil.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import frc.robot.Constants;
import frc.robot.Constants.Slapdown;

/** Hardware IO for the single Kraken slapdown motor, direct driving through a 20:1 reduction. */
public class KrakenSlapdownIO implements SlapdownIO {
  private final TalonFX motor;
  private final TalonFXConfiguration config;

  private final PositionVoltage positionRequest = new PositionVoltage(0.0).withSlot(0);

  private double goalAngleDegrees = 0.0;
  private boolean closedLoop = false;

  // status signals
  private final StatusSignal<Angle> position;
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Current> current;
  private final StatusSignal<Temperature> temperature;

  public KrakenSlapdownIO() {
    motor = new TalonFX(Constants.CAN.kSlapdownMotorID);

    config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.CurrentLimits.StatorCurrentLimit = Slapdown.kStatorCurrentLimitAmps;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    config.Feedback.SensorToMechanismRatio = Slapdown.kGearRatio;
    config.Slot0.kP = Slapdown.kSlapdownP;
    config.Slot0.kI = Slapdown.kSlapdownI;
    config.Slot0.kD = Slapdown.kSlapdownD;

    tryUntilOk(5, () -> motor.getConfigurator().apply(config, 0.25));

    // Create status signals
    position = motor.getPosition();
    velocity = motor.getVelocity();
    current = motor.getStatorCurrent();
    temperature = motor.getDeviceTemp();

    // Configure periodic frames
    BaseStatusSignal.setUpdateFrequencyForAll(50.0, position, velocity, current, temperature);
    motor.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(SlapdownIOInputs inputs) {
    // Refresh all signals
    BaseStatusSignal.refreshAll(position, velocity, current, temperature);

    inputs.currentAngleDegrees = position.getValueAsDouble() * 360.0;
    inputs.velocityRPM = velocity.getValueAsDouble() * 60.0;
    inputs.currentAmps = current.getValueAsDouble();
    inputs.temperatureC = temperature.getValueAsDouble();
    inputs.goalAngleDegrees = goalAngleDegrees;
    inputs.inCloseLoop = closedLoop;
  }

  @Override
  public void setAngle(double angleDegrees) {
    closedLoop = true;
    goalAngleDegrees = angleDegrees;
    motor.setControl(positionRequest.withPosition(angleDegrees / 360.0));
  }

  @Override
  public void setDutyCycle(double dutyCycle) {
    // Open loop, so there is no closed loop setpoint to report
    closedLoop = false;
    motor.set(dutyCycle);
  }

  @Override
  public void setBrakeMode(boolean enable) {
    NeutralModeValue neutralMode = enable ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    config.MotorOutput.NeutralMode = neutralMode;

    // Only the motor output portion of the config needs to be re-sent
    tryUntilOk(5, () -> motor.getConfigurator().apply(config.MotorOutput, 0.25));
  }
}
