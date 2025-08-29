// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.ArmWinch;

import edu.wpi.first.math.controller.PIDController;
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
    ManualControl,
    Zeroing,
    TestPID
  }

  private static PIDController PID;

  private double PIDVoltage = 0.0;
  public WinchStates wantedState = WinchStates.Idle;

  public ArmWinch(ArmWinchIO io) {
    this.io = io;
    PID =
        new PIDController(
            ArmWinchConstants.ControlConstants.kP,
            ArmWinchConstants.ControlConstants.kI,
            ArmWinchConstants.ControlConstants.kD);
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
          if (inputs.armMotorCurrent > 20) {
            wantedState = WinchStates.Idle;
            io.zeroEncoder();
          } else {
            io.setSpeed(-0.3);
          }
          break;
        case TestPID:
          PIDVoltage = PID.calculate(getPosition(), 360);
          io.setVoltage(PIDVoltage);
        default:
          break;
      }
    } else {
      wantedState = WinchStates.Idle;
    }
    Logger.recordOutput("Arm Winch/PID Voltage", PIDVoltage);
    // This method will be called once per scheduler run
  }

  public void setWantedState(WinchStates wantedState) {
    this.wantedState = wantedState;
  }

  public double getPosition() {
    return inputs.thruBorePosition;
  }
}
