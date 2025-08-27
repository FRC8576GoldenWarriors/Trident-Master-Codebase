package frc.robot.Subsystems.ArmWinch;

import org.littletonrobotics.junction.AutoLog;

public interface ArmWinchIO {
  default void updateInputs(ArmWinchInputs inputs) {}

  @AutoLog
  public class ArmWinchInputs {
    public double armMotorVoltage = 0.0;
    public double armMotorCurrent = 0.0;
    public double thruBorePosition = 0.0;
    public double thruBoreVelocity = 0.0;
  }

  default void setVoltage(double voltage) {}

  default void setSpeed(double speed) {}
}
