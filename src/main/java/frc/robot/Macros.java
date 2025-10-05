// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.lang.Thread.State;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Subsystems.ArmPivot.Pivot;
import frc.robot.Subsystems.ArmPivot.PivotConstants;
import frc.robot.Subsystems.ArmPivot.Pivot.PivotPositions;
import frc.robot.Subsystems.ArmWinch.ArmWinch;
import frc.robot.Subsystems.ArmWinch.ArmWinch.WinchStates;
import frc.robot.Subsystems.EndEffector.EndEffector;
import frc.robot.Subsystems.EndEffector.EndEffector.EndEffectorState;

public class Macros extends SubsystemBase {
  /** Creates a new Macrcos. */
  private Pivot armPivot;
  private ArmWinch armWinch;
  private EndEffector endEffector;

  private States wantedState = States.Idle;
  public enum States{
    L4,
    Idle
  }
  public Macros(Pivot armPivot, ArmWinch armWinch, EndEffector endEffector) {
    this.armPivot = armPivot;
    this.armWinch = armWinch;
    this.endEffector = endEffector;
  }

  @Override
  public void periodic() {
    if(DriverStation.isEnabled()){
    // This method will be called once per scheduler run
    switch (wantedState) {
      case L4:
      armPivot.setPivotPosition(PivotPositions.GroundIntake);
        //L4();
        break;
      case Idle:
      armPivot.setPivotPosition(PivotPositions.Idle);
      default:
        break;
    }
  }
  else{
    wantedState = States.Idle;
  }
  Logger.recordOutput("Robot/Wanted State", wantedState);
  }
  public void setWantedState(States wantedState){
    this.wantedState = wantedState;
  }

  private void L4(){
    if(!endEffector.getCoralDetected()){
      
    }
    // if(Math.abs(armPivot.getThruBorePosition()-PivotConstants.ControlConstants.groundIntake)<0.01&&!endEffector.getCoralDetected()){
    //   armWinch.setWantedState(WinchStates.Holding);
    //   endEffector.setWantedState(EndEffectorState.GroundIntake);
    // }
    // if(endEffector.getCoralDetected()){
    //   armPivot.setPivotPosition(PivotPositions.FrontL4);
    //   armWinch.setWantedState(WinchStates.FrontL4);
    //   endEffector.setWantedState(EndEffectorState.L4);
    // }
  }
}
