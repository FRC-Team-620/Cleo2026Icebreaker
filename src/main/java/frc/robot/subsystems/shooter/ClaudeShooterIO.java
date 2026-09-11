package frc.robot.subsystems.shooter;

import static frc.robot.util.PhoenixUtil.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import frc.robot.Constants;

/**
 * Hardware IO for the four Kraken flywheel shooter. Each side is a pulley coupled pair (top leads,
 * bottom follows in the same direction), and the right side is inverted relative to the left so
 * that a positive command shoots on both sides. The flywheels are direct driven, so the rotor and
 * the flywheel turn at the same speed.
 */
public class ClaudeShooterIO implements ShooterIO {
  private final TalonFX leftLeadMotor;
  private final TalonFX leftFollowMotor;
  private final TalonFX rightLeadMotor;
  private final TalonFX rightFollowMotor;

  private final TalonFXConfiguration leftConfig;
  private final TalonFXConfiguration rightConfig;

  private final VelocityVoltage velocityRequest = new VelocityVoltage(0.0).withSlot(0);
  private final VoltageOut voltageRequest = new VoltageOut(0.0);

  private double desiredRPM = 0.0;

  // status signals
  private final StatusSignal<AngularVelocity> leftLeadVelocity;
  private final StatusSignal<Current> leftLeadCurrent;
  private final StatusSignal<Temperature> leftLeadTemperature;

  private final StatusSignal<Current> leftFollowCurrent;
  private final StatusSignal<Temperature> leftFollowTemperature;

  private final StatusSignal<AngularVelocity> rightLeadVelocity;
  private final StatusSignal<Current> rightLeadCurrent;
  private final StatusSignal<Temperature> rightLeadTemperature;

  private final StatusSignal<Current> rightFollowCurrent;
  private final StatusSignal<Temperature> rightFollowTemperature;

  public ClaudeShooterIO() {
    leftLeadMotor = new TalonFX(Constants.CAN.kLeftTopShooterMotorID);
    leftFollowMotor = new TalonFX(Constants.CAN.kLeftBottomShooterMotorID);
    rightLeadMotor = new TalonFX(Constants.CAN.kRightTopShooterMotorID);
    rightFollowMotor = new TalonFX(Constants.CAN.kRightBottomShooterMotorID);

    // Both motors on a side share a config, since the pulley makes them spin together
    leftConfig = new TalonFXConfiguration();
    // leftConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    // leftConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    // leftConfig.CurrentLimits.StatorCurrentLimit = ShooterConstants.kStatorCurrentLimitAmps;
    // leftConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    // leftConfig.Feedback.SensorToMechanismRatio = ShooterConstants.kGearRatio;
    // leftConfig.Slot0.kS = ShooterConstants.kS;
    // leftConfig.Slot0.kV = ShooterConstants.kOnboardV;
    // leftConfig.Slot0.kP = ShooterConstants.kOnboardP;
    // leftConfig.Slot0.kI = ShooterConstants.kOnboardI;
    // leftConfig.Slot0.kD = ShooterConstants.kOnboardD;

    // The right side is mechanically mirrored, so it runs the opposite rotor direction
    rightConfig = new TalonFXConfiguration();
    // rightConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    // rightConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    // rightConfig.CurrentLimits.StatorCurrentLimit = ShooterConstants.kStatorCurrentLimitAmps;
    // rightConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    // rightConfig.Feedback.SensorToMechanismRatio = ShooterConstants.kGearRatio;
    // rightConfig.Slot0.kS = ShooterConstants.kS;
    // rightConfig.Slot0.kV = ShooterConstants.kOnboardV;
    // rightConfig.Slot0.kP = ShooterConstants.kOnboardP;
    // rightConfig.Slot0.kI = ShooterConstants.kOnboardI;
    // rightConfig.Slot0.kD = ShooterConstants.kOnboardD;

    tryUntilOk(5, () -> leftLeadMotor.getConfigurator().apply(leftConfig, 0.25));
    tryUntilOk(5, () -> leftFollowMotor.getConfigurator().apply(leftConfig, 0.25));
    tryUntilOk(5, () -> rightLeadMotor.getConfigurator().apply(rightConfig, 0.25));
    tryUntilOk(5, () -> rightFollowMotor.getConfigurator().apply(rightConfig, 0.25));

    // Both motors of a pair are belted together, so each follower runs with its lead
    // leftFollowMotor.setControl(
    //     new Follower(Constants.CAN.kLeftTopShooterMotorID, MotorAlignmentValue.Aligned));
    rightFollowMotor.setControl(
        new Follower(Constants.CAN.kLeftTopShooterMotorID, MotorAlignmentValue.Opposed));

    // Create status signals. Only the leads need velocity, the followers mirror them
    leftLeadVelocity = leftLeadMotor.getVelocity();
    leftLeadCurrent = leftLeadMotor.getStatorCurrent();
    leftLeadTemperature = leftLeadMotor.getDeviceTemp();

    leftFollowCurrent = leftFollowMotor.getStatorCurrent();
    leftFollowTemperature = leftFollowMotor.getDeviceTemp();

    rightLeadVelocity = rightLeadMotor.getVelocity();
    rightLeadCurrent = rightLeadMotor.getStatorCurrent();
    rightLeadTemperature = rightLeadMotor.getDeviceTemp();

    rightFollowCurrent = rightFollowMotor.getStatorCurrent();
    rightFollowTemperature = rightFollowMotor.getDeviceTemp();

    // Configure periodic frames
    // BaseStatusSignal.setUpdateFrequencyForAll(
    //     50.0,
    //     leftLeadVelocity,
    //     leftLeadCurrent,
    //     leftLeadTemperature,
    //     leftFollowCurrent,
    //     leftFollowTemperature,
    //     rightLeadVelocity,
    //     rightLeadCurrent,
    //     rightLeadTemperature,
    //     rightFollowCurrent,
    //     rightFollowTemperature);
    // ParentDevice.optimizeBusUtilizationForAll(
    //     leftLeadMotor, leftFollowMotor, rightLeadMotor, rightFollowMotor);
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    // Refresh all signals
    BaseStatusSignal.refreshAll(
        leftLeadVelocity,
        leftLeadCurrent,
        leftLeadTemperature,
        leftFollowCurrent,
        leftFollowTemperature,
        rightLeadVelocity,
        rightLeadCurrent,
        rightLeadTemperature,
        rightFollowCurrent,
        rightFollowTemperature);

    // Current is the draw of the whole shooter and temperature is the hottest motor, so a single
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

    // Both sides are commanded to the same speed, so the average is the shooter speed. The
    // inversion is handled by the motor config, so both leads report positive when shooting
    inputs.velocityRPM =
        (leftLeadVelocity.getValueAsDouble() + rightLeadVelocity.getValueAsDouble()) / 2.0 * 60.0;
    inputs.desiredRPM = desiredRPM;
  }

  @Override
  public void setRPM(double rpm) {
    desiredRPM = rpm;
    double rotationsPerSecond = rpm / 60.0;
    // leftLeadMotor.setControl(velocityRequest.withVelocity(rotationsPerSecond));
    // rightLeadMotor.setControl(velocityRequest.withVelocity(rotationsPerSecond));
  }

  @Override
  public void setVoltage(double volts) {
    // Open loop, so there is no closed loop setpoint to report
    desiredRPM = 0.0;
    // leftLeadMotor.setControl(voltageRequest.withOutput(volts));
    // rightLeadMotor.setControl(voltageRequest.withOutput(volts));
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

  public void stop() {
    leftLeadMotor.stopMotor();
  }
}
