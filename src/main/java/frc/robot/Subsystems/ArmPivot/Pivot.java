// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.ArmPivot;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import org.littletonrobotics.junction.Logger;

public class Pivot extends SubsystemBase {
  /** Creates a new Pivot. */
  private PivotIO io;

  private PivotIOInputsAutoLogged inputs = new PivotIOInputsAutoLogged();

  private ProfiledPIDController PID;
  private ArmFeedforward FF;

  private double PIDVoltage;
  private double FFVoltage;
  private double inputVoltage = 0.0;

  public enum PivotPositions {
    Idle,
    Rest,
    FrontL1,
    FrontL2,
    FrontL3,
    FrontL4,
    BackL1,
    BackL2,
    BackL3,
    BackL4,
    GroundIntake,
    StationIntake,
    SetClimb,
    ManualControl,
    ClimbUp,
    ClimbDown
  }

  private PivotPositions wantedPosition = PivotPositions.Idle;

  public Pivot(PivotIO io) {
    this.io = io;
    PID =
        new ProfiledPIDController(
            PivotConstants.ControlConstants.kP,
            PivotConstants.ControlConstants.kI,
            PivotConstants.ControlConstants.kD,
            new Constraints(1.5, 2.0));
      PID.setTolerance(0.03);
    FF =
        new ArmFeedforward(
            PivotConstants.ControlConstants.kS,
            PivotConstants.ControlConstants.kG,
            PivotConstants.ControlConstants.kV,
            PivotConstants.ControlConstants.kA);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Arm Pivot", inputs);
    Logger.recordOutput("Arm Pivot/Position", getThruBorePosition());
    if (DriverStation.isEnabled()) {
      switch (wantedPosition) {
        case Idle:
          io.setVoltage(0);
          break;
        case Rest:
          PIDVoltage =
              PID.calculate(getThruBorePosition(), PivotConstants.ControlConstants.startPosition);
          FFVoltage =
              FF.calculate(
                  (PivotConstants.ControlConstants.startPosition
                          + PivotConstants.ControlConstants.COMOffset)
                      * Math.PI
                      * 2,
                  0.5);
          inputVoltage = PIDVoltage + FFVoltage;
          io.setVoltage(inputVoltage);
          break;
        case FrontL1:
          PIDVoltage =
              PID.calculate(getThruBorePosition(), PivotConstants.ControlConstants.frontL1);
          FFVoltage =
              -FF.calculate(
                  (PivotConstants.ControlConstants.frontL1
                          + PivotConstants.ControlConstants.COMOffset)
                      * Math.PI
                      * 2,
                  0.5);
          inputVoltage = PIDVoltage + FFVoltage;
          io.setVoltage(inputVoltage);
          break;
        case FrontL2:
          PIDVoltage =
              PID.calculate(getThruBorePosition(), PivotConstants.ControlConstants.frontL2);
          FFVoltage =
              FF.calculate(
                  (PivotConstants.ControlConstants.frontL2
                          + PivotConstants.ControlConstants.COMOffset)
                      * Math.PI
                      * 2,
                  0.5);
          inputVoltage = PIDVoltage + FFVoltage;
          io.setVoltage(inputVoltage);
          break;
        case FrontL3:
          PIDVoltage =
              PID.calculate(getThruBorePosition(), PivotConstants.ControlConstants.frontL3);
          FFVoltage =
              FF.calculate(
                  (PivotConstants.ControlConstants.frontL3
                          + PivotConstants.ControlConstants.COMOffset)
                      * Math.PI
                      * 2,
                  0.5);
          inputVoltage = PIDVoltage + FFVoltage;
          io.setVoltage(inputVoltage);
          break;
        case FrontL4:
          PIDVoltage =
              PID.calculate(getThruBorePosition(), PivotConstants.ControlConstants.frontL4);
          FFVoltage =
              FF.calculate(
                  (PivotConstants.ControlConstants.frontL4
                          + PivotConstants.ControlConstants.COMOffset)
                      * Math.PI
                      * 2,
                  0.5);
          inputVoltage = PIDVoltage + FFVoltage;
          io.setVoltage(inputVoltage);
          break;
        case BackL1:
          PIDVoltage = PID.calculate(getThruBorePosition(), PivotConstants.ControlConstants.backL1);
          FFVoltage =
              FF.calculate(
                  (PivotConstants.ControlConstants.backL1
                          + PivotConstants.ControlConstants.COMOffset)
                      * Math.PI
                      * 2,
                  0.5);
          inputVoltage = PIDVoltage + FFVoltage;
          io.setVoltage(inputVoltage);
          break;
        case BackL2:
          PIDVoltage = PID.calculate(getThruBorePosition(), PivotConstants.ControlConstants.backL2);
          FFVoltage =
              FF.calculate(
                  (PivotConstants.ControlConstants.backL2
                          + PivotConstants.ControlConstants.COMOffset)
                      * Math.PI
                      * 2,
                  0.5);
          inputVoltage = PIDVoltage + FFVoltage;
          io.setVoltage(inputVoltage);
          break;
        case BackL3:
          PIDVoltage = PID.calculate(getThruBorePosition(), PivotConstants.ControlConstants.backL3);
          FFVoltage =
              FF.calculate(
                  (PivotConstants.ControlConstants.backL3
                          + PivotConstants.ControlConstants.COMOffset)
                      * Math.PI
                      * 2,
                  0.5);
          inputVoltage = PIDVoltage + FFVoltage;
          io.setVoltage(inputVoltage);
          break;
        case BackL4:
          PIDVoltage = PID.calculate(getThruBorePosition(), PivotConstants.ControlConstants.backL4);
          FFVoltage =
              FF.calculate(
                  (PivotConstants.ControlConstants.backL4
                          + PivotConstants.ControlConstants.COMOffset)
                      * Math.PI
                      * 2,
                  0.5);
          inputVoltage = PIDVoltage + FFVoltage;
          io.setVoltage(inputVoltage);
          break;
        case GroundIntake:
          PIDVoltage =
              PID.calculate(getThruBorePosition(), PivotConstants.ControlConstants.groundIntake);
          FFVoltage =
              FF.calculate(
                  (PivotConstants.ControlConstants.groundIntake
                          + PivotConstants.ControlConstants.COMOffset)
                      * Math.PI
                      * 2,
                  0.5);
          inputVoltage = PIDVoltage + FFVoltage;
          io.setVoltage(inputVoltage);
          break;
        case StationIntake:
          PIDVoltage =
              PID.calculate(getThruBorePosition(), PivotConstants.ControlConstants.stationIntake);
          FFVoltage =
              FF.calculate(
                  (PivotConstants.ControlConstants.stationIntake
                          + PivotConstants.ControlConstants.COMOffset)
                      * Math.PI
                      * 2,
                  0.5);
          inputVoltage = PIDVoltage + FFVoltage;
          io.setVoltage(inputVoltage);
          break;
        case SetClimb:
          PIDVoltage =
              PID.calculate(getThruBorePosition(), PivotConstants.ControlConstants.setClimb);
          FFVoltage =
              FF.calculate(
                  (PivotConstants.ControlConstants.setClimb
                          + PivotConstants.ControlConstants.COMOffset)
                      * Math.PI
                      * 2,
                  0.5);
          inputVoltage = PIDVoltage + FFVoltage;
          io.setVoltage(inputVoltage);
          break;
        case ClimbUp:
          PIDVoltage =
              PID.calculate(getThruBorePosition(), PivotConstants.ControlConstants.climbUp);
          FFVoltage =
              FF.calculate(
                  (PivotConstants.ControlConstants.climbUp
                          + PivotConstants.ControlConstants.COMOffset)
                      * Math.PI
                      * 2,
                  0.5);
          inputVoltage = PIDVoltage + FFVoltage;
          io.setVoltage(inputVoltage);
          break;
        case ClimbDown:
          PIDVoltage =
              PID.calculate(getThruBorePosition(), PivotConstants.ControlConstants.climbDown);
          FFVoltage =
              FF.calculate(
                  (PivotConstants.ControlConstants.climbDown
                          + PivotConstants.ControlConstants.COMOffset)
                      * Math.PI
                      * 2,
                  0.5);
          inputVoltage = PIDVoltage + FFVoltage;
          io.setVoltage(inputVoltage);
          break;
        case ManualControl:
          if (RobotContainer.driverController.povUp().getAsBoolean()) {
            io.setSpeed(0.5);
          } else if (RobotContainer.driverController.povDown().getAsBoolean()) {
            io.setSpeed(-0.5);
          } else {
            wantedPosition = PivotPositions.Rest;
          }
          break;
        default:
          break;
      }
    } else {
      wantedPosition = PivotPositions.Idle;
    }
    Logger.recordOutput("Arm Pivot/Input Voltage", inputVoltage);
    // This method will be called once per scheduler run
  }

  public void setPivotPosition(PivotPositions wantedPosition) {
    this.wantedPosition = wantedPosition;
  }

  public double getThruBorePosition() {
    return inputs.thruBorePosition;
  }

  public PivotPositions getPosition() {
    return wantedPosition;
  }
}
