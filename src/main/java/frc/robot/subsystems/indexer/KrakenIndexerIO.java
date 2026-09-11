package frc.robot.subsystems.indexer;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.Constants;

public class KrakenIndexerIO implements IndexerIO {

  private TalonFX leadMotor;
  private TalonFX blMotor;
  private TalonFX frMotor;
  private TalonFX brMotor;

  private TalonFXConfiguration leadConfig;
  private TalonFXConfiguration followConfig;

  // status signals

  public KrakenIndexerIO() {
    leadMotor = new TalonFX(Constants.CAN.kFrontLeftIndexerMotorID);
    blMotor = new TalonFX(Constants.CAN.kBackLeftIndexerMotorID);
    frMotor = new TalonFX(Constants.CAN.kFrontRightIndexerMotorID);
    brMotor = new TalonFX(Constants.CAN.kBackRightIndexerMotorID);
  }

  @Override
  public void updateInputs(IndexerIOInputs inputs) {
    // Update inputs from the Kraken indexer hardware
  }

  @Override
  public void setSpeed(double speed) {
    // Set the speed of the Kraken indexer motor
  }

  @Override
  public void setBrakeMode(boolean enable) {
    // Enable or disable brake mode for the Kraken indexer motor
  }
}
