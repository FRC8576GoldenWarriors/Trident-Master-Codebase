package frc.robot.Subsystems.EndEffector;

import com.ctre.phoenix6.hardware.CANrange;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.lib.drivers.WarriorSparkMax;

public class EndEffectorIOSparkMax implements EndEffectorIO {
  private WarriorSparkMax sideRollerMotor;
  private WarriorSparkMax topRollerMotor;
  private WarriorSparkMax pivotMotor;

  private DutyCycleEncoder pivotThruBore;
  private CANrange coralSensor;

  public EndEffectorIOSparkMax() {
    sideRollerMotor =
        new WarriorSparkMax(
            EndEffectorConstants.HardwareConstants.sideRollerID,
            MotorType.kBrushless,
            EndEffectorConstants.HardwareConstants.sideRollerInverted,
            IdleMode.kCoast,
            EndEffectorConstants.HardwareConstants.sideRollerCurrentLimit);
    topRollerMotor =
        new WarriorSparkMax(
            EndEffectorConstants.HardwareConstants.topRollerID,
            MotorType.kBrushless,
            EndEffectorConstants.HardwareConstants.topRollerInverted,
            IdleMode.kCoast,
            EndEffectorConstants.HardwareConstants.topRollerCurrentLimit);
    pivotMotor =
        new WarriorSparkMax(
            EndEffectorConstants.HardwareConstants.pivotMotorID,
            MotorType.kBrushless,
            EndEffectorConstants.HardwareConstants.pivotMotorInverted,
            IdleMode.kCoast,
            EndEffectorConstants.HardwareConstants.pivotMotorCurrentLimit);

    pivotThruBore =
        new DutyCycleEncoder(
            EndEffectorConstants.HardwareConstants.pivotThruBoreID,
            EndEffectorConstants.HardwareConstants.thruBoreFullRange,
            EndEffectorConstants.HardwareConstants.thruBoreExpectedZero);
    coralSensor = new CANrange(EndEffectorConstants.HardwareConstants.canRangeID);
  }

  @Override
  public void updateInputs(EndEffectorInputs inputs) {
    inputs.sideRollerVoltage = sideRollerMotor.getAppliedOutput();
    inputs.sideRollerCurrent = sideRollerMotor.getOutputCurrent();
    inputs.topRollerVoltage = topRollerMotor.getAppliedOutput();
    inputs.topRollerCurrent = topRollerMotor.getOutputCurrent();

    inputs.pivotVoltage = pivotMotor.getOutputCurrent();
    inputs.pivotCurrent = pivotMotor.getOutputCurrent();

    inputs.coralDetected = coralSensor.getIsDetected().asSupplier().get();
    inputs.rangeConnected = coralSensor.isConnected();

    inputs.thruBorePosition = pivotThruBore.get();
    inputs.thruBoreConnected = pivotThruBore.isConnected();
  }

  @Override
  public void setPivotVoltage(double pivotVoltage) {
    pivotMotor.setVoltage(pivotVoltage);
  }

  @Override
  public void setRollerVoltages(double topRollerVoltage, double sideRollerVoltage) {
    topRollerMotor.setVoltage(topRollerVoltage);
    sideRollerMotor.setVoltage(sideRollerVoltage);
  }

  @Override
  public void setRollerSpeeds(double topRollerSpeed, double sideRollerSpeed) {
    topRollerMotor.set(topRollerSpeed);
    sideRollerMotor.set(sideRollerSpeed);
  }

  @Override
  public void setPivotSpeed(double pivotSpeed) {
    pivotMotor.set(pivotSpeed);
  }
}
