package frc.robot.Subsystems.ArmWinch;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.lib.drivers.WarriorSparkMax;

public class ArmWinchIOSparkMax implements ArmWinchIO {
  private WarriorSparkMax winchMax;
  private DutyCycleEncoder absEncoder;

  public ArmWinchIOSparkMax() {
    winchMax =
        new WarriorSparkMax(
            ArmWinchConstants.HardwareConstants.pivotSparkMaxID,
            MotorType.kBrushless,
            ArmWinchConstants.HardwareConstants.motorInverted,
            IdleMode.kCoast,
            ArmWinchConstants.HardwareConstants.currentLimit);
    absEncoder =
        new DutyCycleEncoder(
            ArmWinchConstants.HardwareConstants.thruBoreID,
            ArmWinchConstants.HardwareConstants.thruBoreFullRange,
            ArmWinchConstants.HardwareConstants.thruBoreExpectedZero);
    absEncoder.setInverted(ArmWinchConstants.HardwareConstants.thruBoreInverted);
  }

  @Override
  public void updateInputs(ArmWinchInputs inputs) {
    inputs.armMotorVoltage = winchMax.getAppliedOutput();
    inputs.armMotorCurrent = winchMax.getOutputCurrent();
    inputs.thruBorePosition = absEncoder.get();
    inputs.thruBoreConnected = absEncoder.isConnected();
  }

  @Override
  public void setVoltage(double voltage) {
    winchMax.setVoltage(voltage);
  }

  @Override
  public void setSpeed(double speed) {
    winchMax.set(speed);
  }
}
