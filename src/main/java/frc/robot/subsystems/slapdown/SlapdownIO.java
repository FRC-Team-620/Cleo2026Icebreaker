package frc.robot.subsystems.slapdown;

import org.littletonrobotics.junction.AutoLog;

public interface SlapdownIO {
  @AutoLog
  public static class SlapdownIOInputs {
    /** Current mechanism angle. */
    public double currentAngleDegrees;

    /** Mechanism speed. */
    public double velocityRPM;

    /** Motor stator current. */
    public double currentAmps;

    /** Motor temperature. */
    public double temperatureC;

    /** Closed loop setpoint, zero whenever the slapdown is being run open loop. */
    public double goalAngleDegrees;
  }

  public default void updateInputs(SlapdownIOInputs inputs) {}

  /** Closed loop position control. */
  public default void setAngle(double angleDegrees) {}

  /** Open loop duty cycle control. */
  public default void setDutyCycle(double dutyCycle) {}

  public default void setBrakeMode(boolean enable) {}
}
