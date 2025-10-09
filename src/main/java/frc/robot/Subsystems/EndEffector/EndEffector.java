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
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class EndEffector extends SubsystemBase {
  /** Creates a new EndEffector. */
  private EndEffectorIO io;

  private EndEffectorInputsAutoLogged inputs = new EndEffectorInputsAutoLogged();

  public enum EndEffectorState {
    Idle,
    RollerVoltageControl,
    PivotVoltageControl,
    GroundIntake,
    GroundIntakeHold,
    Hold,
    L1,
    L1Score,
    L2,
    L2Score,
    L3,
    L3Score,
    L4,
    L4Back,
    L4Score,
    L4BackScore
  }

  private EndEffectorState wantedState;

  private ProfiledPIDController PID;
  private ArmFeedforward FF;

  private double PIDVoltage;
  private double FFVoltage;
  private double inputVoltage = 0.0;

  private LoggedNetworkNumber kPLoggedNetworkNumber =
      new LoggedNetworkNumber("Tuning/End Effector kP", EndEffectorConstants.ControlConstants.kP);
  private double kP = kPLoggedNetworkNumber.get();

  public EndEffector(EndEffectorIO io) {
    PID =
        new ProfiledPIDController(
            EndEffectorConstants.ControlConstants.kP,
            EndEffectorConstants.ControlConstants.kI,
            EndEffectorConstants.ControlConstants.kD,
            new Constraints(5, 10.75)); // 0.5, 1
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
    // 340 Last kP Checked. Slow
    Logger.processInputs("End Effector", inputs);
    kP = kPLoggedNetworkNumber.get();
    PID.setP(kP);
    if (DriverStation.isEnabled()) {
      switch (wantedState) {
        case Idle:
          io.setPivotVoltage(0);
          io.setRollerVoltages(0);
          break;
        case RollerVoltageControl:
          if (RobotContainer.driverController.povRight().getAsBoolean()) {
            io.setRollerSpeeds(0.5);
          } else if (RobotContainer.driverController.povLeft().getAsBoolean()) {
            io.setRollerSpeeds(-0.5);
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
          if (Math.abs(getPosition() - EndEffectorConstants.ControlConstants.groundIntakePosition)
              < 0.05) {
            PID.setP(50);
          }
          PIDVoltage =
              PID.calculate(
                  getPosition(), EndEffectorConstants.ControlConstants.groundIntakePosition);
          FFVoltage =
              -FF.calculate(
                  (EndEffectorConstants.ControlConstants.groundIntakePosition
                          + EndEffectorConstants.ControlConstants.COMOffset)
                      * 2
                      * Math.PI,
                  0.5);
          inputVoltage = PIDVoltage + FFVoltage;
          io.setPivotVoltage(inputVoltage);
          io.setRollerSpeeds(0.9);
          break;
        case GroundIntakeHold:
          if (Math.abs(getPosition() - EndEffectorConstants.ControlConstants.groundIntakePosition)
              < 0.05) {
            PID.setP(50);
          }
          PIDVoltage =
              PID.calculate(
                  getPosition(), EndEffectorConstants.ControlConstants.groundIntakePosition);
          FFVoltage =
              -FF.calculate(
                  (EndEffectorConstants.ControlConstants.groundIntakePosition
                          + EndEffectorConstants.ControlConstants.COMOffset)
                      * 2
                      * Math.PI,
                  0.5);
          inputVoltage = PIDVoltage + FFVoltage;
          io.setPivotVoltage(inputVoltage);
          io.setRollerSpeeds(0.08); // 0.1
          break;
        case L1:
          if (Math.abs(getPosition() - EndEffectorConstants.ControlConstants.l1Position) < 0.05) {
            PID.setP(50);
          }
          PIDVoltage =
              PID.calculate(getPosition(), EndEffectorConstants.ControlConstants.l1Position);
          FFVoltage =
              -FF.calculate(
                  (EndEffectorConstants.ControlConstants.l1Position
                          + EndEffectorConstants.ControlConstants.COMOffset)
                      * 2
                      * Math.PI,
                  0.5);
          inputVoltage = PIDVoltage + FFVoltage;
          io.setPivotVoltage(inputVoltage);
          break;
        case L4:
          if (Math.abs(getPosition() - EndEffectorConstants.ControlConstants.l4Position) < 0.05) {
            PID.setP(50);
          }
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
        case L1Score:
          if (Math.abs(getPosition() - EndEffectorConstants.ControlConstants.l1Position) < 0.05) {
            PID.setP(50);
          }
          PIDVoltage =
              PID.calculate(getPosition(), EndEffectorConstants.ControlConstants.l1Position);
          FFVoltage =
              -FF.calculate(
                  (EndEffectorConstants.ControlConstants.l1Position
                          + EndEffectorConstants.ControlConstants.COMOffset)
                      * 2
                      * Math.PI,
                  0.5);
          inputVoltage = PIDVoltage + FFVoltage;
          io.setPivotVoltage(inputVoltage);
          io.setRollerSpeeds(0.9);
          break;

        case L2:
          if (Math.abs(getPosition() - EndEffectorConstants.ControlConstants.l2Position) < 0.05) {
            PID.setP(50);
          }
          PIDVoltage =
              PID.calculate(getPosition(), EndEffectorConstants.ControlConstants.l2Position);
          FFVoltage =
              -FF.calculate(
                  (EndEffectorConstants.ControlConstants.l2Position
                          + EndEffectorConstants.ControlConstants.COMOffset)
                      * 2
                      * Math.PI,
                  0.5);
          inputVoltage = PIDVoltage + FFVoltage;
          io.setPivotVoltage(inputVoltage);
          break;
        case L2Score:
          if (Math.abs(getPosition() - EndEffectorConstants.ControlConstants.l2Position) < 0.05) {
            PID.setP(50);
          }
          PIDVoltage =
              PID.calculate(getPosition(), EndEffectorConstants.ControlConstants.l2Position);
          FFVoltage =
              -FF.calculate(
                  (EndEffectorConstants.ControlConstants.l2Position
                          + EndEffectorConstants.ControlConstants.COMOffset)
                      * 2
                      * Math.PI,
                  0.5);
          inputVoltage = PIDVoltage + FFVoltage;
          io.setPivotVoltage(inputVoltage);
          io.setRollerSpeeds(0.9);
          break;

        case L3:
          if (Math.abs(getPosition() - EndEffectorConstants.ControlConstants.l3Position) < 0.05) {
            PID.setP(50);
          }
          PIDVoltage =
              PID.calculate(getPosition(), EndEffectorConstants.ControlConstants.l3Position);
          FFVoltage =
              -FF.calculate(
                  (EndEffectorConstants.ControlConstants.l3Position
                          + EndEffectorConstants.ControlConstants.COMOffset)
                      * 2
                      * Math.PI,
                  0.5);
          inputVoltage = PIDVoltage + FFVoltage;
          io.setPivotVoltage(inputVoltage);
          break;
        case L3Score:
          if (Math.abs(getPosition() - EndEffectorConstants.ControlConstants.l3Position) < 0.05) {
            PID.setP(50);
          }
          PIDVoltage =
              PID.calculate(getPosition(), EndEffectorConstants.ControlConstants.l3Position);
          FFVoltage =
              -FF.calculate(
                  (EndEffectorConstants.ControlConstants.l3Position
                          + EndEffectorConstants.ControlConstants.COMOffset)
                      * 2
                      * Math.PI,
                  0.5);
          inputVoltage = PIDVoltage + FFVoltage;
          io.setPivotVoltage(inputVoltage);
          io.setRollerSpeeds(0.9);
          break;
        case Hold:
          if (Math.abs(getPosition() - EndEffectorConstants.ControlConstants.holdPosition) < 0.05) {
            PID.setP(50);
          }
          PIDVoltage =
              PID.calculate(getPosition(), EndEffectorConstants.ControlConstants.holdPosition);
          FFVoltage =
              -FF.calculate(
                  (EndEffectorConstants.ControlConstants.holdPosition
                          + EndEffectorConstants.ControlConstants.COMOffset)
                      * 2
                      * Math.PI,
                  0.5);
          inputVoltage = PIDVoltage + FFVoltage;
          io.setPivotVoltage(inputVoltage);
          io.setRollerSpeeds(0.08); // 0.1
          break;
        case L4Score:
          if (Math.abs(getPosition() - EndEffectorConstants.ControlConstants.l4Position) < 0.05) {
            PID.setP(50);
          }
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
          io.setRollerSpeeds(0.9);
          break;
        case L4Back:
          if (Math.abs(getPosition() - EndEffectorConstants.ControlConstants.l4BackPosition)
              < 0.05) {
            PID.setP(50);
          }
          PIDVoltage =
              PID.calculate(getPosition(), EndEffectorConstants.ControlConstants.l4BackPosition);
          FFVoltage =
              -FF.calculate(
                  (EndEffectorConstants.ControlConstants.l4BackPosition
                          + EndEffectorConstants.ControlConstants.COMOffset)
                      * 2
                      * Math.PI,
                  0.5);
          inputVoltage = PIDVoltage + FFVoltage;
          io.setPivotVoltage(inputVoltage);
          break;
        case L4BackScore:
          if (Math.abs(getPosition() - EndEffectorConstants.ControlConstants.l4BackPosition)
              < 0.05) {
            PID.setP(50);
          }
          PIDVoltage =
              PID.calculate(getPosition(), EndEffectorConstants.ControlConstants.l4BackPosition);
          FFVoltage =
              -FF.calculate(
                  (EndEffectorConstants.ControlConstants.l4BackPosition
                          + EndEffectorConstants.ControlConstants.COMOffset)
                      * 2
                      * Math.PI,
                  0.5);
          inputVoltage = PIDVoltage + FFVoltage;
          io.setPivotVoltage(inputVoltage);
          io.setRollerSpeeds(0.9);
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
    Logger.recordOutput("End Effector/kP", PID.getP());
    Logger.recordOutput("End Effector/Wanted State", wantedState);
    // This method will be called once per scheduler run
  }

  public void setWantedState(EndEffectorState wantedState) {
    if (this.wantedState != wantedState) PID.reset(getPosition());
    this.wantedState = wantedState;
  }

  public double getPosition() {
    return inputs.thruBorePosition;
  }

  public boolean getCoralDetected() {
    return inputs.coralDetected;
  }

  public boolean endEffectorInRange() {
    return Math.abs(getPosition() - PID.getGoal().position) < 0.03;
  }
}
