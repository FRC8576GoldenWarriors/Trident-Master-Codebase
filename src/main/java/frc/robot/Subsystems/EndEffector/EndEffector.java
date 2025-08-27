// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.EndEffector;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import org.littletonrobotics.junction.Logger;

public class EndEffector extends SubsystemBase {
  /** Creates a new EndEffector. */
  private EndEffectorIO io;

  private EndEffectorInputsAutoLogged inputs = new EndEffectorInputsAutoLogged();

  public enum EndEffectorState {
    Idle,
    RollerVoltageControl,
    PivotVoltageControl
  }

  private EndEffectorState wantedState;

  public EndEffector(EndEffectorIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("End Effector", inputs);
    if (DriverStation.isEnabled()) {
      switch (wantedState) {
        case Idle:
          io.setPivotVoltage(0);
          io.setRollerVoltages(0, 0);
          break;
        case RollerVoltageControl:
          if (RobotContainer.driverController.x().getAsBoolean()) {
            io.setRollerSpeeds(0.3, 0.3);
          } else if (RobotContainer.driverController.a().getAsBoolean()) {
            io.setRollerSpeeds(-.3, -.3);
          } else {
            wantedState = EndEffectorState.Idle;
          }
          break;
        case PivotVoltageControl:
          if (RobotContainer.driverController.leftBumper().getAsBoolean()) {
            io.setPivotSpeed(-0.3);
          } else if (RobotContainer.driverController.rightBumper().getAsBoolean()) {
            io.setPivotSpeed(0.3);
          } else {
            wantedState = EndEffectorState.Idle;
          }
          break;

        default:
          break;
      }
    } else {
      wantedState = EndEffectorState.Idle;
    }
    // This method will be called once per scheduler run
  }

  public void setWantedState(EndEffectorState wantedState) {
    this.wantedState = wantedState;
  }
}
