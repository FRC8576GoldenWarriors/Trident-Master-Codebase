// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.ArmWinch;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import org.littletonrobotics.junction.Logger;

public class ArmWinch extends SubsystemBase {
  /** Creates a new ArmWinch. */
  private ArmWinchIO io;

  private ArmWinchInputsAutoLogged inputs = new ArmWinchInputsAutoLogged();

  public enum WinchStates {
    Idle,
    Holding,
    ManualControl,
    Zeroing,
    TestPID
  }

  private ProfiledPIDController PID;
  private ArmFeedforward FF;

  private double PIDVoltage = 0.0;
  public WinchStates wantedState = WinchStates.Idle;

  public ArmWinch(ArmWinchIO io) {
    this.io = io;
    PID =
        new ProfiledPIDController(
            ArmWinchConstants.ControlConstants.kP,
            ArmWinchConstants.ControlConstants.kI,
            ArmWinchConstants.ControlConstants.kD,
            new Constraints(
                ArmWinchConstants.ControlConstants.maxVelocity,
                ArmWinchConstants.ControlConstants.maxAcceleration));

    FF = new ArmFeedforward(0.0, ArmWinchConstants.ControlConstants.kG, 0.0);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Arm Winch", inputs);
    Logger.recordOutput("Arm Winch/Position", getPosition());
    SmartDashboard.putNumber("Winch Position", getPosition());
    if (DriverStation.isEnabled()) {
      switch (wantedState) {
        case Idle:
          io.setVoltage(0);
          break;
        case Holding:
          PIDVoltage = PID.calculate(getPosition(), new TrapezoidProfile.State(100, 0.0));
          io.setVoltage(PIDVoltage);
          break;
        case ManualControl:
          if (RobotContainer.driverController.a().getAsBoolean()) {
            io.setSpeed(0.3);
          } else if (RobotContainer.driverController.y().getAsBoolean()) {
            io.setSpeed(-0.3);
          } else {
            wantedState = WinchStates.Idle;
          }
          break;
        case Zeroing:
          if (inputs.armMotorCurrent > 22) { // 22.5
            io.zeroEncoder();
            wantedState = WinchStates.Holding;
          } else {
            io.setSpeed(-0.3);
          }
          break;
        case TestPID:
        if(Math.abs(getPosition()-2000)<100){
          PID.setP(0.5);
        }
          PIDVoltage = PID.calculate(getPosition(), new TrapezoidProfile.State(2000, 0.0));
          io.setVoltage(PIDVoltage);
          break;
        default:
          break;
      }
    } else {
      wantedState = WinchStates.Idle;
    }
    Logger.recordOutput("Arm Winch/PID Voltage", PIDVoltage);
    Logger.recordOutput("Arm Winch/Wanted State", wantedState);
    Logger.recordOutput("Arm Winch/PID At Goal", PID.atGoal());

    // This method will be called once per scheduler run
  }

  public void setWantedState(WinchStates wantedState) {
    this.wantedState = wantedState;
  }

  public double getPosition() {
    return inputs.thruBorePosition;
  }
  public void zeroEncoder(){
    io.zeroEncoder();
  }
}
