package frc.robot.Subsystems.ArmPivot;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.lib.drivers.WarriorSparkMax;

public class PivotIOSparkMax implements PivotIO {

  private WarriorSparkMax pivotMotor;
  private DutyCycleEncoder thruBore;

  public PivotIOSparkMax() {
    pivotMotor =
        new WarriorSparkMax(
            PivotConstants.HardwareConstants.pivotSparkMaxID,
            MotorType.kBrushless,
            PivotConstants.HardwareConstants.motorInverted,
            IdleMode.kCoast,
            PivotConstants.HardwareConstants.currentLimit);
    thruBore =
        new DutyCycleEncoder(
            PivotConstants.HardwareConstants.thruBoreID,
            PivotConstants.HardwareConstants.thruBoreFullRange,
            PivotConstants.HardwareConstants.thruBoreExpectedZero);

    // thruBore.setAssumedFrequenPcy(966);
    // thruBore.setConnectedFrequencyThreshold(35);
    thruBore.setInverted(PivotConstants.HardwareConstants.thruBoreInverted);
  }

  @Override
  public void updateInputs(PivotIOInputs inputs) {
    inputs.motorVoltage = pivotMotor.getAppliedOutput();
    inputs.motorCurrent = pivotMotor.getOutputCurrent();
    inputs.thruBorePosition = thruBore.get();
    inputs.thruBoreConnected = thruBore.isConnected();
  }

  @Override
  public void setVoltage(double voltage) {
    pivotMotor.setVoltage(voltage);
  }

  @Override
  public void setSpeed(double speed) {
    pivotMotor.set(speed);
  }
}
