package frc.robot.Subsystems.ArmPivot;

import org.littletonrobotics.junction.AutoLog;

public interface PivotIO {

  default void updateInputs(PivotIOInputs inputs) {}

  @AutoLog
  public class PivotIOInputs {
    public double motorVoltage = 0.0;
    public double motorCurrent = 0.0;
    public double thruBorePosition = 0.0;
    public boolean thruBoreConnected = false;
    public double thruBoreVelocity = 0.0;
  }

  default void setVoltage(double voltage) {}

  default void setSpeed(double speed) {}
}
