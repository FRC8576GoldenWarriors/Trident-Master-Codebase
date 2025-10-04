// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.EndEffector;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
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
    PivotVoltageControl,
    GroundIntake,
    L4
  }

  private EndEffectorState wantedState;

  private ProfiledPIDController PID;
  private ArmFeedforward FF;

  private double PIDVoltage;
  private double FFVoltage;
  private double inputVoltage = 0.0;

  public EndEffector(EndEffectorIO io) {
    PID =
        new ProfiledPIDController(
            EndEffectorConstants.ControlConstants.kP,
            EndEffectorConstants.ControlConstants.kI,
            EndEffectorConstants.ControlConstants.kD,
            new Constraints(0.5, 0.75));
    FF =
        new ArmFeedforward(
            EndEffectorConstants.ControlConstants.kS,
            EndEffectorConstants.ControlConstants.kG,
            EndEffectorConstants.ControlConstants.kV,
            EndEffectorConstants.ControlConstants.kA);
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
          io.setRollerVoltages(0);
          break;
        case RollerVoltageControl:
          if (RobotContainer.driverController.povRight().getAsBoolean()) {
            io.setRollerSpeeds(0.25);
          } else if (RobotContainer.driverController.povLeft().getAsBoolean()) {
            io.setRollerSpeeds(-0.25);
          } else {
            wantedState = EndEffectorState.Idle;
          }
          break;
        case PivotVoltageControl:
          if (RobotContainer.driverController.leftBumper().getAsBoolean()) {
            io.setPivotSpeed(-0.05);
          } else if (RobotContainer.driverController.leftTrigger().getAsBoolean()) {
            io.setPivotSpeed(0.05);
          } else {
            wantedState = EndEffectorState.Idle;
          }
          break;
        case GroundIntake:
          PIDVoltage =
              PID.calculate(
                  getPosition(), EndEffectorConstants.ControlConstants.groundIntakePosition);
          FFVoltage =
              -FF.calculate(
                  EndEffectorConstants.ControlConstants.groundIntakePosition * 2 * Math.PI, 0.5);
          inputVoltage = PIDVoltage + FFVoltage;
          io.setPivotVoltage(inputVoltage);
          break;
        case L4:
          PIDVoltage =
              PID.calculate(getPosition(), EndEffectorConstants.ControlConstants.l4Position);
          FFVoltage =
              -FF.calculate(
                  (EndEffectorConstants.ControlConstants.l4Position
                          + EndEffectorConstants.ControlConstants.COMOffset)
                      * 2
                      * Math.PI,
                  0.5);
          inputVoltage = PIDVoltage + FFVoltage;
          io.setPivotVoltage(inputVoltage);
          break;
        default:
          break;
      }
    } else {
      wantedState = EndEffectorState.Idle;
    }
    Logger.recordOutput("End Effector/PID Voltage", PIDVoltage);
    Logger.recordOutput("End Effector/FF Voltage", FFVoltage);
    Logger.recordOutput("End Effector/Input Voltage", inputVoltage);
    // This method will be called once per scheduler run
  }

  public void setWantedState(EndEffectorState wantedState) {
    PID.reset(getPosition());
    this.wantedState = wantedState;
  }

  public double getPosition() {
    return inputs.thruBorePosition;
  }
}
