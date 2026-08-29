package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
  @AutoLog
  public static class ShooterIOInputs {
    /** Stator current summed across all four flywheel motors. */
    public double currentAmps;

    /** Measured flywheel speed, averaged across the left and right lead motors. */
    public double velocityRPM;

    /** Temperature of the hottest of the four motors. */
    public double temperatureC;

    /** Closed loop setpoint, zero whenever the shooter is being run open loop. */
    public double desiredRPM;
  }

  public default void updateInputs(ShooterIOInputs inputs) {}

  /** Closed loop velocity control. Positive spins the flywheels in the shooting direction. */
  public default void setRPM(double rpm) {}

  /** Open loop voltage control. Positive spins the flywheels in the shooting direction. */
  public default void setVoltage(double volts) {}

  public default void setBrakeMode(boolean enable) {}

  public default void stop() {}
}
