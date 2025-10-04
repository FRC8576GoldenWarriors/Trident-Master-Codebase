package frc.robot.Subsystems.EndEffector;

import org.littletonrobotics.junction.AutoLog;

public interface EndEffectorIO {
  default void updateInputs(EndEffectorInputs inputs) {}

  @AutoLog
  public class EndEffectorInputs {
    public double sideRollerVoltage = 0.0;
    public double sideRollerCurrent = 0.0;
    public double topRollerVoltage = 0.0;
    public double topRollerCurrent = 0.0;

    public double pivotVoltage = 0.0;
    public double pivotCurrent = 0.0;

    public boolean coralDetected = false;
    public boolean rangeConnected = false;

    public double thruBorePosition = 0.0;
    public boolean thruBoreConnected = false;
  }

  default void setRollerVoltages(double sideRollerVoltage) {}

  default void setRollerSpeeds(double sideRollerSpeed) {}

  default void setPivotVoltage(double pivotVoltage) {}

  default void setPivotSpeed(double pivotSpeed) {}
}
