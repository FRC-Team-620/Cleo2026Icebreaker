package frc.robot.subsystems.shooter;

import static frc.robot.util.PhoenixUtil.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;

public class LeftKrakenShooterIO implements ShooterIO {
  private TalonFX leadMotor;
  private TalonFX leftBottomMotor;
  // private TalonFX rightTopMotor;
  // private TalonFX rightBottomMotor;

  private TalonFXConfiguration config;

  public LeftKrakenShooterIO() {
    leadMotor = new TalonFX(Constants.CAN.kLeftTopShooterMotorID);
    leadMotor.getConfigurator().apply(new TalonFXConfiguration());

    leftBottomMotor = new TalonFX(Constants.CAN.kLeftBottomShooterMotorID);
    leftBottomMotor.getConfigurator().apply(new TalonFXConfiguration());

    config = new TalonFXConfiguration();
    // config.slot0.kP = ShooterConstants.kP;
    // config.slot0.kI = ShooterConstants.kI;
    // config.slot0.kD = ShooterConstants.kD;
    // config.slot0.kF = ShooterConstants.kF;
    // config.slot0.integralZone = ShooterConstants.kIZone;
    // config.slot0.closedLoopPeakOutput = ShooterConstants.kMaxOutput;
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.CurrentLimits.StatorCurrentLimit = ShooterConstants.kStatorCurrentLimitAmps;
    config.CurrentLimits.StatorCurrentLimitEnable = true;

    // leadMotor.getConfigurator().apply(config);
    // leftBottomMotor.getConfigurator().apply(config);
    // rightTopMotor.getConfigurator().apply(config);
    // rightBottomMotor.getConfigurator().apply(config);

    tryUntilOk(5, () -> leadMotor.getConfigurator().apply(config, 0.25));
    tryUntilOk(5, () -> leftBottomMotor.getConfigurator().apply(config, 0.25));

    // Set followers
    leftBottomMotor.setControl(
        new Follower(Constants.CAN.kLeftTopShooterMotorID, MotorAlignmentValue.Aligned));
  }

  public void setVoltage(double volts) {
    leadMotor.setVoltage(volts);
  }

  public void stop() {
    leadMotor.stopMotor();
  }
}
