// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.ArmWinch;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class ArmWinch extends SubsystemBase {
  /** Creates a new ArmWinch. */
  private ArmWinchIO io;

  private ArmWinchInputsAutoLogged inputs = new ArmWinchInputsAutoLogged();

  public enum WinchStates {
    Idle,
    Holding,
    ManualControl,
    Zeroing,
    TestPID,
    FrontL1,
    FrontL2,
    FrontL3,
    FrontL4,
    BackL1,
    BackL2,
    BackL3,
    BackL4,
    GroundIntake,
    StationIntake
  }

  private ProfiledPIDController PID;
  private ElevatorFeedforward FF;

  private LoggedNetworkNumber kPLoggedNetworkNumber =
      new LoggedNetworkNumber("Tuning/Arm Winch kP", ArmWinchConstants.ControlConstants.kP);
  private LoggedNetworkNumber kGLoggedNetworkNumber =
      new LoggedNetworkNumber("Tuning/Arm Winch kG", ArmWinchConstants.ControlConstants.kG);
  private double kP = kPLoggedNetworkNumber.get();
  private double kG = kGLoggedNetworkNumber.get();
  private double PIDVoltage = 0.0;
  private double FFVoltage = 0.0;
  private double inputVoltage = 0.0;
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

    FF =
        new ElevatorFeedforward(
            ArmWinchConstants.ControlConstants.kS,
            ArmWinchConstants.ControlConstants.kG,
            ArmWinchConstants.ControlConstants.kV,
            ArmWinchConstants.ControlConstants.kA);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Arm Winch", inputs);
    kP = kPLoggedNetworkNumber.get();
    PID.setP(kP);
    kG = kGLoggedNetworkNumber.get();
    FF.setKg(kG);
    Logger.recordOutput("Arm Winch/Position", getPosition());
    SmartDashboard.putNumber("Winch Position", getPosition());
    if (DriverStation.isEnabled()) {
      switch (wantedState) {
        case Idle:
          io.setVoltage(0);
          break;
        case Holding:
        if (Math.abs(getPosition() - ArmWinchConstants.ControlConstants.holding) < 200) {
            PID.setP(0.08);
          }
          PIDVoltage =
              PID.calculate(
                  getPosition(),
                  new TrapezoidProfile.State(ArmWinchConstants.ControlConstants.holding, 0.0));
          io.setVoltage(PIDVoltage);
          break;
        case ManualControl:
          if (RobotContainer.driverController.a().getAsBoolean()) {
            io.setSpeed(0.05);
          } else if (RobotContainer.driverController.y().getAsBoolean()) {
            io.setSpeed(-0.05);
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
          // 0.064 is smooth
          if (Math.abs(getPosition() - ArmWinchConstants.ControlConstants.testPosition) < 50
              || getPosition() > ArmWinchConstants.ControlConstants.testPosition) {
            PID.setP(0.128);
          }
          PIDVoltage =
              PID.calculate(
                  getPosition(),
                  new TrapezoidProfile.State(ArmWinchConstants.ControlConstants.testPosition, 0.0));

          FFVoltage = FF.calculate(ArmWinchConstants.ControlConstants.maxVelocity);
          inputVoltage = PIDVoltage + FFVoltage;

          io.setVoltage(inputVoltage);
          break;
        case FrontL1:
          if (Math.abs(getPosition() - ArmWinchConstants.ControlConstants.frontL1) < 100
              || getPosition() > ArmWinchConstants.ControlConstants.frontL1) {
            // PID.setP(0.2);
          }
          PIDVoltage =
              PID.calculate(
                  getPosition(),
                  new TrapezoidProfile.State(ArmWinchConstants.ControlConstants.frontL1, 0.0));
          io.setVoltage(PIDVoltage);
          break;
        case FrontL2:
          if (Math.abs(getPosition() - ArmWinchConstants.ControlConstants.frontL2) < 100
              || getPosition() > ArmWinchConstants.ControlConstants.frontL2) {
            // PID.setP(0.2);
          }
          PIDVoltage =
              PID.calculate(
                  getPosition(),
                  new TrapezoidProfile.State(ArmWinchConstants.ControlConstants.frontL2, 0.0));
          io.setVoltage(PIDVoltage);
          break;
        case FrontL3:
          if (Math.abs(getPosition() - ArmWinchConstants.ControlConstants.frontL3) < 100
              || getPosition() > ArmWinchConstants.ControlConstants.frontL3) {
            // PID.setP(0.2);
          }
          PIDVoltage =
              PID.calculate(
                  getPosition(),
                  new TrapezoidProfile.State(ArmWinchConstants.ControlConstants.frontL3, 0.0));
          io.setVoltage(PIDVoltage);
          break;
        case FrontL4:
          if (Math.abs(getPosition() - ArmWinchConstants.ControlConstants.frontL4) < 200) {
             PID.setP(0.08);
          }
          PIDVoltage =
              PID.calculate(
                  getPosition(),
                  new TrapezoidProfile.State(ArmWinchConstants.ControlConstants.frontL4, 0.0));
          io.setVoltage(PIDVoltage);
          break;
        case BackL1:
          if (Math.abs(getPosition() - ArmWinchConstants.ControlConstants.backL1) < 100
              || getPosition() > ArmWinchConstants.ControlConstants.backL1) {
            // PID.setP(0.2);
          }
          PIDVoltage =
              PID.calculate(
                  getPosition(),
                  new TrapezoidProfile.State(ArmWinchConstants.ControlConstants.backL1, 0.0));
          io.setVoltage(PIDVoltage);
          break;
        case BackL2:
          if (Math.abs(getPosition() - ArmWinchConstants.ControlConstants.backL2) < 100
              || getPosition() > ArmWinchConstants.ControlConstants.backL2) {
            // PID.setP(0.2);
          }
          PIDVoltage =
              PID.calculate(
                  getPosition(),
                  new TrapezoidProfile.State(ArmWinchConstants.ControlConstants.backL2, 0.0));
          io.setVoltage(PIDVoltage);
          break;
        case BackL3:
          if (Math.abs(getPosition() - ArmWinchConstants.ControlConstants.backL3) < 100
              || getPosition() > ArmWinchConstants.ControlConstants.backL3) {
            // PID.setP(0.2);
          }
          PIDVoltage =
              PID.calculate(
                  getPosition(),
                  new TrapezoidProfile.State(ArmWinchConstants.ControlConstants.backL3, 0.0));
          io.setVoltage(PIDVoltage);
          break;
        case BackL4:
          if (Math.abs(getPosition() - ArmWinchConstants.ControlConstants.backL4) < 100
              || getPosition() > ArmWinchConstants.ControlConstants.backL4) {
            // PID.setP(0.2);
          }
          PIDVoltage =
              PID.calculate(
                  getPosition(),
                  new TrapezoidProfile.State(ArmWinchConstants.ControlConstants.backL4, 0.0));
          io.setVoltage(PIDVoltage);
          break;
        case GroundIntake:
          if (Math.abs(getPosition() - ArmWinchConstants.ControlConstants.groundIntake) < 100
              || getPosition() > ArmWinchConstants.ControlConstants.groundIntake) {
            // PID.setP(0.2);
          }
          PIDVoltage =
              PID.calculate(
                  getPosition(),
                  new TrapezoidProfile.State(ArmWinchConstants.ControlConstants.groundIntake, 0.0));
          io.setVoltage(PIDVoltage);
          break;
        case StationIntake:
          if (Math.abs(getPosition() - ArmWinchConstants.ControlConstants.stationIntake) < 100
              || getPosition() > ArmWinchConstants.ControlConstants.stationIntake) {
            // PID.setP(0.2);
          }
          PIDVoltage =
              PID.calculate(
                  getPosition(),
                  new TrapezoidProfile.State(
                      ArmWinchConstants.ControlConstants.stationIntake, 0.0));
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
    Logger.recordOutput("Arm Winch/Input Voltage", inputVoltage);

    // This method will be called once per scheduler run
  }

  public void setWantedState(WinchStates wantedState) {
    if(this.wantedState!=wantedState)
    PID.reset(getPosition());
    this.wantedState = wantedState;
  }

  public double getPosition() {
    return inputs.thruBorePosition;
  }

  public double getVelocity() {
    return inputs.thruBoreVelocity;
  }

  public void zeroEncoder() {
    io.zeroEncoder();
  }
}
