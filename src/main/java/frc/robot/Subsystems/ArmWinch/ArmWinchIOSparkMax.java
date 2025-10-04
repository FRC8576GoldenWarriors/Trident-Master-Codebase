package frc.robot.Subsystems.ArmWinch;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.wpilibj.Encoder;
import frc.lib.drivers.WarriorSparkMax;

public class ArmWinchIOSparkMax implements ArmWinchIO {
  private WarriorSparkMax winchMax;
  private Encoder absEncoder;

  public ArmWinchIOSparkMax() {
    winchMax =
        new WarriorSparkMax(
            ArmWinchConstants.HardwareConstants.pivotSparkMaxID,
            MotorType.kBrushless,
            ArmWinchConstants.HardwareConstants.motorInverted,
            IdleMode.kCoast,
            ArmWinchConstants.HardwareConstants.currentLimit);
    absEncoder =
        new Encoder(
            ArmWinchConstants.HardwareConstants.encoderPort1,
            ArmWinchConstants.HardwareConstants.encoderPort2);

    absEncoder.setDistancePerPulse(360);
  }

  @Override
  public void updateInputs(ArmWinchInputs inputs) {
    inputs.armMotorVoltage = winchMax.getAppliedOutput();
    inputs.armMotorCurrent = winchMax.getOutputCurrent();
    inputs.thruBorePosition = absEncoder.getDistance() / 2048;
    // inputs.thruBoreVelocity = absEncoder.getRate()/2048;
  }

  @Override
  public void setVoltage(double voltage) {
    winchMax.setVoltage(voltage);
  }

  @Override
  public void setSpeed(double speed) {
    winchMax.set(speed);
  }

  @Override
  public void zeroEncoder() {
    absEncoder.reset();
  }
}
