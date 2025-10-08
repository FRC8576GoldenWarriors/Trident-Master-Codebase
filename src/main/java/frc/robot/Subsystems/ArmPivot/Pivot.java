// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.ArmPivot;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.RobotContainer;
import frc.robot.Subsystems.ArmPivot.Pivot.PivotPositions;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class Pivot extends SubsystemBase {
  /** Creates a new Pivot. */
  private PivotIO io;

  private PivotIOInputsAutoLogged inputs = new PivotIOInputsAutoLogged();

  private ProfiledPIDController PID;
  private ArmFeedforward FF;

  private double PIDVoltage;
  private double FFVoltage;
  private double inputVoltage = 0.0;
  private LoggedNetworkNumber kPLoggedNetworkNumber =
      new LoggedNetworkNumber("/Tuning/Arm Pivot kP", PivotConstants.ControlConstants.kP);
  private double kP = kPLoggedNetworkNumber.get();
  // 256
  private SysIdRoutine routine =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              null, null, null, (state) -> Logger.recordOutput("SysIdState", state.toString())),
          new SysIdRoutine.Mechanism(
              (voltage) -> io.setVoltage(voltage.in(Units.Volts)), null, this));

  public enum PivotPositions {
    Idle,
    Rest,
    FrontL1,
    FrontL2,
    FrontL3,
    FrontL4,
    // BackL1,
    // BackL2,
    // BackL3,
    BackL4,
    GroundIntake,
    StationIntake,
    SetClimb,
    ManualControl,
    ClimbUp,
    ClimbDown,
    CloseHold
  }

  private PivotPositions wantedPosition = PivotPositions.Idle;

  public Pivot(PivotIO io) {
    this.io = io;
    PID =
        new ProfiledPIDController(
            PivotConstants.ControlConstants.kP,
            PivotConstants.ControlConstants.kI,
            PivotConstants.ControlConstants.kD,
            new Constraints(15, 20.0));
    PID.setTolerance(0.002);
    PID.setIZone(0.025);
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
    kP = kPLoggedNetworkNumber.get();
    PID.setP(kP);
    Logger.recordOutput("Arm Pivot/Position", getThruBorePosition());
    Logger.recordOutput("Arm Pivot/kP", PID.getP());
    if (DriverStation.isEnabled()) {
      switch (wantedPosition) {
        case Idle:
          io.setVoltage(0);
          break;
        case Rest:
          if (Math.abs(getThruBorePosition() - PivotConstants.ControlConstants.startingPosition)
              < 0.05) {
            PID.setP(50);
          }
          PIDVoltage =
              PID.calculate(
                  getThruBorePosition(), PivotConstants.ControlConstants.startingPosition);
          FFVoltage =
              -FF.calculate(
                  (PivotConstants.ControlConstants.startingPosition
                          + PivotConstants.ControlConstants.COMOffset)
                      * Math.PI
                      * 2,
                  0.5);
          inputVoltage = PIDVoltage + FFVoltage;
          io.setVoltage(inputVoltage);
          break;
        case FrontL1:
          if (Math.abs(getThruBorePosition() - PivotConstants.ControlConstants.frontL1) < 0.05) {
            PID.setP(50);
          }
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
          if (Math.abs(getThruBorePosition() - PivotConstants.ControlConstants.frontL2) < 0.05) {
            PID.setP(50);
          }
          PIDVoltage =
              PID.calculate(getThruBorePosition(), PivotConstants.ControlConstants.frontL2);
          FFVoltage =
              -FF.calculate(
                  (PivotConstants.ControlConstants.frontL2
                          + PivotConstants.ControlConstants.COMOffset)
                      * Math.PI
                      * 2,
                  0.5);
          inputVoltage = PIDVoltage + FFVoltage;
          io.setVoltage(inputVoltage);
          break;
        case FrontL3:
          if (Math.abs(getThruBorePosition() - PivotConstants.ControlConstants.frontL3) < 0.05) {
            PID.setP(50);
          }
          PIDVoltage =
              PID.calculate(getThruBorePosition(), PivotConstants.ControlConstants.frontL3);
          FFVoltage =
              -FF.calculate(
                  (PivotConstants.ControlConstants.frontL3
                          + PivotConstants.ControlConstants.COMOffset)
                      * Math.PI
                      * 2,
                  0.5);
          inputVoltage = PIDVoltage + FFVoltage;
          io.setVoltage(inputVoltage);
          break;
        case FrontL4:
          if (Math.abs(getThruBorePosition() - PivotConstants.ControlConstants.frontL4) < 0.05) {
            PID.setP(50);
          }
          PIDVoltage =
              PID.calculate(getThruBorePosition(), PivotConstants.ControlConstants.frontL4);
          FFVoltage =
              -FF.calculate(
                  (PivotConstants.ControlConstants.frontL4
                          + PivotConstants.ControlConstants.COMOffset)
                      * Math.PI
                      * 2,
                  0.5);
          inputVoltage = PIDVoltage + FFVoltage;
          io.setVoltage(inputVoltage);
          break;
          // case BackL1:
          //   PIDVoltage = PID.calculate(getThruBorePosition(),
          // PivotConstants.ControlConstants.backL1);
          //   FFVoltage =
          //       -FF.calculate(
          //           (PivotConstants.ControlConstants.backL1
          //                   + PivotConstants.ControlConstants.COMOffset)
          //               * Math.PI
          //               * 2,
          //           0.5);
          //   inputVoltage = PIDVoltage + FFVoltage;
          //   io.setVoltage(inputVoltage);
          //   break;
          // case BackL2:
          //   PIDVoltage = PID.calculate(getThruBorePosition(),
          // PivotConstants.ControlConstants.backL2);
          //   FFVoltage =
          //       -FF.calculate(
          //           (PivotConstants.ControlConstants.backL2
          //                   + PivotConstants.ControlConstants.COMOffset)
          //               * Math.PI
          //               * 2,
          //           0.5);
          //   inputVoltage = PIDVoltage + FFVoltage;
          //   io.setVoltage(inputVoltage);
          //   break;
          // case BackL3:
          //   PIDVoltage = PID.calculate(getThruBorePosition(),
          // PivotConstants.ControlConstants.backL3);
          //   FFVoltage =
          //       -FF.calculate(
          //           (PivotConstants.ControlConstants.backL3
          //                   + PivotConstants.ControlConstants.COMOffset)
          //               * Math.PI
          //               * 2,
          //           0.5);
          //   inputVoltage = PIDVoltage + FFVoltage;
          //   io.setVoltage(inputVoltage);
          //   break;
        case BackL4:
          PIDVoltage = PID.calculate(getThruBorePosition(), PivotConstants.ControlConstants.backL4);
          FFVoltage =
              -FF.calculate(
                  (PivotConstants.ControlConstants.backL4
                          + PivotConstants.ControlConstants.COMOffset)
                      * Math.PI
                      * 2,
                  0.5);
          inputVoltage = PIDVoltage + FFVoltage;
          io.setVoltage(inputVoltage);
          break;
        case GroundIntake:
          if (Math.abs(getThruBorePosition() - PivotConstants.ControlConstants.groundIntake)
              < 0.05) {
            PID.setP(50);
          }
          PIDVoltage =
              PID.calculate(getThruBorePosition(), PivotConstants.ControlConstants.groundIntake);
          FFVoltage =
              -FF.calculate(
                  (PivotConstants.ControlConstants.groundIntake
                          + PivotConstants.ControlConstants.COMOffset)
                      * Math.PI
                      * 2,
                  0.5);
          inputVoltage = PIDVoltage + FFVoltage;
          io.setVoltage(inputVoltage);
          break;
        case StationIntake:
          if (Math.abs(getThruBorePosition() - PivotConstants.ControlConstants.stationIntake)
              < 0.05) {
            PID.setP(50);
          }
          PIDVoltage =
              PID.calculate(getThruBorePosition(), PivotConstants.ControlConstants.stationIntake);
          FFVoltage =
              -FF.calculate(
                  (PivotConstants.ControlConstants.stationIntake
                          + PivotConstants.ControlConstants.COMOffset)
                      * Math.PI
                      * 2,
                  0.5);
          inputVoltage = PIDVoltage + FFVoltage;
          io.setVoltage(inputVoltage);
          break;
        case SetClimb:
          if (Math.abs(getThruBorePosition() - PivotConstants.ControlConstants.setClimb) < 0.05) {
            PID.setP(50);
          }
          PIDVoltage =
              PID.calculate(getThruBorePosition(), PivotConstants.ControlConstants.setClimb);
          FFVoltage =
              -FF.calculate(
                  (PivotConstants.ControlConstants.setClimb
                          + PivotConstants.ControlConstants.COMOffset)
                      * Math.PI
                      * 2,
                  0.5);
          inputVoltage = PIDVoltage + FFVoltage;
          io.setVoltage(inputVoltage);
          break;
        case ClimbUp:
          if (Math.abs(getThruBorePosition() - PivotConstants.ControlConstants.climbUp) < 0.05) {
            PID.setP(50);
          }
          PIDVoltage =
              PID.calculate(getThruBorePosition(), PivotConstants.ControlConstants.climbUp);
          FFVoltage =
              -FF.calculate(
                  (PivotConstants.ControlConstants.climbUp
                          + PivotConstants.ControlConstants.COMOffset)
                      * Math.PI
                      * 2,
                  0.5);
          inputVoltage = PIDVoltage + FFVoltage;
          io.setVoltage(inputVoltage);
          break;
        case ClimbDown:
          if (Math.abs(getThruBorePosition() - PivotConstants.ControlConstants.climbDown) < 0.05) {
            PID.setP(50);
          }
          PIDVoltage =
              PID.calculate(getThruBorePosition(), PivotConstants.ControlConstants.climbDown);
          FFVoltage =
              -FF.calculate(
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
            io.setSpeed(0.05);
          } else if (RobotContainer.driverController.povDown().getAsBoolean()) {
            io.setSpeed(-0.05);
          } else {
            wantedPosition = PivotPositions.Idle;
          }
          break;
        case CloseHold:
          if (Math.abs(getThruBorePosition() - PivotConstants.ControlConstants.closeHold) < 0.05) {
            PID.setP(50);
          }
          PIDVoltage =
              PID.calculate(getThruBorePosition(), PivotConstants.ControlConstants.closeHold);
          FFVoltage =
              -FF.calculate(
                  (PivotConstants.ControlConstants.closeHold
                          + PivotConstants.ControlConstants.COMOffset)
                      * Math.PI
                      * 2,
                  0.5);
          inputVoltage = PIDVoltage + FFVoltage;
          io.setVoltage(inputVoltage);
          break;
        default:
          break;
      }
    } else {
      wantedPosition = PivotPositions.Idle;
    }
    Logger.recordOutput("Arm Pivot/Input Voltage", inputVoltage);
    Logger.recordOutput("Arm Pivot/PID At Goal", PID.atGoal());
    Logger.recordOutput("Arm Pivot/Pivot Position", wantedPosition);
    Logger.recordOutput("Arm Pivot/Goal Position", PID.getGoal().position);
    Logger.recordOutput("Arm Pivot/PID Voltage", PIDVoltage);
    Logger.recordOutput("Arm Pivot/FF Voltage", FFVoltage);
    Logger.recordOutput("Arm Pivot/Position in threshold", positionInThreshold());
    // This method will be called once per scheduler run
  }

  public void setPivotPosition(PivotPositions wantedPosition) {
    if (this.wantedPosition != wantedPosition) {
      PID.reset(getThruBorePosition(), getThruBoreVelocity());
      PID.setConstraints(new Constraints(15, 20));
    }
    this.wantedPosition = wantedPosition;
  }

  public double getThruBorePosition() {
    return inputs.thruBorePosition;
  }

  public double getThruBoreVelocity() {
    return inputs.thruBoreVelocity;
  }

  public PivotPositions getPosition() {
    return wantedPosition;
  }

  public Command qualitisticRoutine(Direction direction) {
    if (direction.equals(Direction.kForward))
      return routine.quasistatic(direction).until(() -> getThruBorePosition() > .3);
    return routine.quasistatic(direction);
  }

  public Command dynamicRoutine(Direction direction) {
    if (direction.equals(Direction.kForward))
      return routine.dynamic(direction).until(() -> getThruBorePosition() > .3);
    return routine.dynamic(direction);
  }

  public boolean positionInThreshold() {
    return (Math.abs(getThruBorePosition() - PID.getGoal().position)) < 0.005; // 0.016
  }
}
