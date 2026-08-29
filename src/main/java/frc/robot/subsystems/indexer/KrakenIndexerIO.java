package frc.robot.subsystems.indexer;

import static frc.robot.util.PhoenixUtil.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import frc.robot.Constants;
import frc.robot.Constants.Indexer;

/**
 * Hardware IO for the four Kraken indexer motors. Each side is a pulley coupled pair (front leads,
 * back follows in the same direction), and the right side is inverted relative to the left so that
 * a positive command feeds on both sides. The indexer is direct driven, so the rotor and the
 * mechanism turn at the same speed.
 */
public class KrakenIndexerIO implements IndexerIO {
  private final TalonFX leftLeadMotor;
  private final TalonFX leftFollowMotor;
  private final TalonFX rightLeadMotor;
  private final TalonFX rightFollowMotor;

  private final TalonFXConfiguration leftConfig;
  private final TalonFXConfiguration rightConfig;

  // status signals
  private final StatusSignal<AngularVelocity> leftLeadVelocity;
  private final StatusSignal<Current> leftLeadCurrent;
  private final StatusSignal<Temperature> leftLeadTemperature;

  private final StatusSignal<Current> leftFollowCurrent;
  private final StatusSignal<Temperature> leftFollowTemperature;

  private final StatusSignal<Current> rightLeadCurrent;
  private final StatusSignal<Temperature> rightLeadTemperature;

  private final StatusSignal<Current> rightFollowCurrent;
  private final StatusSignal<Temperature> rightFollowTemperature;

  public KrakenIndexerIO() {
    leftLeadMotor = new TalonFX(Constants.CAN.kFrontLeftIndexerMotorID);
    leftFollowMotor = new TalonFX(Constants.CAN.kBackLeftIndexerMotorID);
    rightLeadMotor = new TalonFX(Constants.CAN.kFrontRightIndexerMotorID);
    rightFollowMotor = new TalonFX(Constants.CAN.kBackRightIndexerMotorID);

    // Both motors on a side share a config, since the pulley makes them spin together
    leftConfig = new TalonFXConfiguration();
    leftConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    leftConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    leftConfig.CurrentLimits.StatorCurrentLimit = Indexer.kStatorCurrentLimitAmps;
    leftConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    leftConfig.Feedback.SensorToMechanismRatio = Indexer.kGearRatio;

    // The right side is mechanically mirrored, so it runs the opposite rotor direction
    rightConfig = new TalonFXConfiguration();
    rightConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    rightConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    rightConfig.CurrentLimits.StatorCurrentLimit = Indexer.kStatorCurrentLimitAmps;
    rightConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    rightConfig.Feedback.SensorToMechanismRatio = Indexer.kGearRatio;

    tryUntilOk(5, () -> leftLeadMotor.getConfigurator().apply(leftConfig, 0.25));
    tryUntilOk(5, () -> leftFollowMotor.getConfigurator().apply(leftConfig, 0.25));
    tryUntilOk(5, () -> rightLeadMotor.getConfigurator().apply(rightConfig, 0.25));
    tryUntilOk(5, () -> rightFollowMotor.getConfigurator().apply(rightConfig, 0.25));

    // Both motors of a pair are belted together, so each follower runs with its lead
    leftFollowMotor.setControl(
        new Follower(Constants.CAN.kFrontLeftIndexerMotorID, MotorAlignmentValue.Aligned));
    rightFollowMotor.setControl(
        new Follower(Constants.CAN.kFrontRightIndexerMotorID, MotorAlignmentValue.Aligned));

    // Create status signals. Only the left lead needs velocity, the rest mirror it
    leftLeadVelocity = leftLeadMotor.getVelocity();
    leftLeadCurrent = leftLeadMotor.getStatorCurrent();
    leftLeadTemperature = leftLeadMotor.getDeviceTemp();

    leftFollowCurrent = leftFollowMotor.getStatorCurrent();
    leftFollowTemperature = leftFollowMotor.getDeviceTemp();

    rightLeadCurrent = rightLeadMotor.getStatorCurrent();
    rightLeadTemperature = rightLeadMotor.getDeviceTemp();

    rightFollowCurrent = rightFollowMotor.getStatorCurrent();
    rightFollowTemperature = rightFollowMotor.getDeviceTemp();

    // Configure periodic frames
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        leftLeadVelocity,
        leftLeadCurrent,
        leftLeadTemperature,
        leftFollowCurrent,
        leftFollowTemperature,
        rightLeadCurrent,
        rightLeadTemperature,
        rightFollowCurrent,
        rightFollowTemperature);
    ParentDevice.optimizeBusUtilizationForAll(
        leftLeadMotor, leftFollowMotor, rightLeadMotor, rightFollowMotor);
  }

  @Override
  public void updateInputs(IndexerIOInputs inputs) {
    // Refresh all signals
    BaseStatusSignal.refreshAll(
        leftLeadVelocity,
        leftLeadCurrent,
        leftLeadTemperature,
        leftFollowCurrent,
        leftFollowTemperature,
        rightLeadCurrent,
        rightLeadTemperature,
        rightFollowCurrent,
        rightFollowTemperature);

    // Current is the draw of the whole indexer and temperature is the hottest motor, so a single
    // overheating or stalling motor is still visible in the logs
    inputs.currentAmps =
        leftLeadCurrent.getValueAsDouble()
            + leftFollowCurrent.getValueAsDouble()
            + rightLeadCurrent.getValueAsDouble()
            + rightFollowCurrent.getValueAsDouble();
    inputs.temperatureC =
        Math.max(
            Math.max(
                leftLeadTemperature.getValueAsDouble(), leftFollowTemperature.getValueAsDouble()),
            Math.max(
                rightLeadTemperature.getValueAsDouble(),
                rightFollowTemperature.getValueAsDouble()));

    // The inversion is handled by the motor config, so the lead reports positive when feeding
    inputs.velocityRPM = leftLeadVelocity.getValueAsDouble() * 60.0;
  }

  @Override
  public void setDutyCycle(double dutyCycle) {
    leftLeadMotor.set(dutyCycle);
    rightLeadMotor.set(dutyCycle);
  }

  @Override
  public void setBrakeMode(boolean enable) {
    NeutralModeValue neutralMode = enable ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    leftConfig.MotorOutput.NeutralMode = neutralMode;
    rightConfig.MotorOutput.NeutralMode = neutralMode;

    // Only the motor output portion of the config needs to be re-sent
    tryUntilOk(5, () -> leftLeadMotor.getConfigurator().apply(leftConfig.MotorOutput, 0.25));
    tryUntilOk(5, () -> leftFollowMotor.getConfigurator().apply(leftConfig.MotorOutput, 0.25));
    tryUntilOk(5, () -> rightLeadMotor.getConfigurator().apply(rightConfig.MotorOutput, 0.25));
    tryUntilOk(5, () -> rightFollowMotor.getConfigurator().apply(rightConfig.MotorOutput, 0.25));
  }
}
