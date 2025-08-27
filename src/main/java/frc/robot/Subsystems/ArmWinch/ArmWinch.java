// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.ArmWinch;

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
    ManualControl
  }

  public WinchStates wantedState = WinchStates.Idle;

  public ArmWinch(ArmWinchIO io) {
    this.io = io;
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
        default:
          break;
      }
    } else {
      wantedState = WinchStates.Idle;
    }
    // This method will be called once per scheduler run
  }

  public void setWantedState(WinchStates wantedState) {
    this.wantedState = wantedState;
  }

  public double getPosition() {
    return inputs.thruBorePosition;
  }
}
