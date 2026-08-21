// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs {
    public double motorVoltage;
    public double motorCurrentAMPS;
    public double motorTemperatureC;
    public double motorVelocityRPM;
  }

  public default void updateInputs(IntakeIOInputs inputs) {}

  public default void setSpeedDutyCycle(double dutyCycle) {}

  public default void setBrakeMode(boolean enable) {}
}
