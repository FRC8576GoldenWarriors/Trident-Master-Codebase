// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import static edu.wpi.first.units.Units.Ohm;

import java.lang.System.Logger.Level;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.config.RobotConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Subsystems.ArmPivot.Pivot;
import frc.robot.Subsystems.ArmPivot.PivotConstants;
import frc.robot.Subsystems.ArmPivot.Pivot.PivotPositions;
import frc.robot.Subsystems.ArmWinch.ArmWinch;
import frc.robot.Subsystems.ArmWinch.ArmWinch.WinchStates;
import frc.robot.Subsystems.EndEffector.EndEffector;
import frc.robot.Subsystems.EndEffector.EndEffector.EndEffectorState;

public class Macros extends SubsystemBase {
  private Pivot m_ArmPivot;
  private ArmWinch m_Winch;
  private EndEffector m_EndEffector;
  public enum states{
    Idle,
    GroundIntake,
    L1,
    L2,
    L3,
    L4,
    L4Back,
    Score
  }

  
  private states wantedState = states.Idle;
  
  /** Creates a new Macros. */
  public Macros(Pivot m_ArmPivot, ArmWinch m_Winch, EndEffector m_EndEffector) {
    this.m_ArmPivot = m_ArmPivot;
    this.m_Winch = m_Winch;
    this.m_EndEffector = m_EndEffector;
  }

  @Override
  public void periodic() {
    if(DriverStation.isEnabled()){
    switch (wantedState) {
      case GroundIntake:
        groundIntake();
        break;
      case L1:
        L1();
        break;
      case L2:
        L2();
        break;
      case L3:
        L3();
        break;
      case L4:
        L4();
        break;
      case L4Back:
        L4Back();
        break;
      case Score:
        score();
        break;
      default:
        break;
    }
  }
  else{
    wantedState = states.Idle;
  }
  Logger.recordOutput("Robot/Wanted State", wantedState);
  
    // This method will be called once per scheduler run
  }
  private void groundIntake(){
    if(!m_EndEffector.getCoralDetected()){
    m_ArmPivot.setPivotPosition(PivotPositions.GroundIntake);
    
    //m_EndEffector.setWantedState(EndEffectorState.Hold);
    // m_Winch.setWantedState(WinchStates.GroundIntake);
    // m_EndEffector.setWantedState(EndEffectorState.GroundIntake);
    }
    if(m_ArmPivot.positionInThreshold()&&m_ArmPivot.getPosition()==PivotPositions.GroundIntake){
      //m_ArmPivot.setPivotPosition(PivotPositions.CloseHold);
      m_EndEffector.setWantedState(EndEffectorState.GroundIntake);
      m_Winch.setWantedState(WinchStates.Holding);
      // m_EndEffector.setWantedState(EndEffectorState.GroundIntake);
      // m_Winch.setWantedState(WinchStates.Holding);
    }
    if(m_EndEffector.getCoralDetected()&&m_ArmPivot.getPosition()==PivotPositions.GroundIntake){
      m_ArmPivot.setPivotPosition(PivotPositions.Rest);
      m_EndEffector.setWantedState(EndEffectorState.Hold);
    }
    // if(m_ArmPivot.positionInThreshold()&&m_ArmPivot.getPosition()==PivotPositions.CloseHold&&m_EndEffector.getCoralDetected()){
    //   m_EndEffector.setWantedState(EndEffectorState.Hold);
    // }
    // if(m_EndEffector.endEffectorInRange()&&m_EndEffector.getCoralDetected()){
    //   m_ArmPivot.setPivotPosition(PivotPositions.Rest);
    // }



    // if(m_EndEffector.getCoralDetected()){
    //   //m_EndEffector.setWantedState(EndEffectorState.Idle);
    //   m_ArmPivot.setPivotPosition(PivotPositions.Rest);
    //   m_EndEffector.setWantedState(EndEffectorState.Hold);
    //}
    // if(m_EndEffector.getCoralDetected()){
    //   m_ArmPivot.setPivotPosition(PivotPositions.FrontL4);
    // }
  }

  private void L1(){
    m_ArmPivot.setPivotPosition(PivotPositions.FrontL1);
    m_EndEffector.setWantedState(EndEffectorState.L1);
  }

  private void L2(){
    m_ArmPivot.setPivotPosition(PivotPositions.FrontL2);
    m_EndEffector.setWantedState(EndEffectorState.L2);
  }
  private void L3(){
    m_ArmPivot.setPivotPosition(PivotPositions.FrontL3);
    m_EndEffector.setWantedState(EndEffectorState.L3);
    m_Winch.setWantedState(WinchStates.FrontL3);
  }

  private void L4(){
    m_ArmPivot.setPivotPosition(PivotPositions.FrontL4);
    if(m_ArmPivot.positionInThreshold()){
    m_EndEffector.setWantedState(EndEffectorState.L4);
    m_Winch.setWantedState(WinchStates.FrontL4);
  }
}

  private void L4Back(){
    m_ArmPivot.setPivotPosition(PivotPositions.BackL4);
    if(m_ArmPivot.positionInThreshold()){
    m_EndEffector.setWantedState(EndEffectorState.L4Back);
    m_Winch.setWantedState(WinchStates.BackL4);
    }
  }

  private void score(){
    if(m_ArmPivot.getPosition()==PivotPositions.FrontL4){
      if(m_EndEffector.getCoralDetected()){
      m_EndEffector.setWantedState(EndEffectorState.L4Score);
      }
      if(!m_EndEffector.getCoralDetected()){
        m_EndEffector.setWantedState(EndEffectorState.Hold);
        m_ArmPivot.setPivotPosition(PivotPositions.Rest);
        m_Winch.setWantedState(WinchStates.Holding);
      }
    }
    if(m_ArmPivot.getPosition()==PivotPositions.FrontL1){
      if(m_EndEffector.getCoralDetected()){
      m_EndEffector.setWantedState(EndEffectorState.L1Score);
      }
      if(!m_EndEffector.getCoralDetected()){
        m_EndEffector.setWantedState(EndEffectorState.Hold);
        m_ArmPivot.setPivotPosition(PivotPositions.Rest);
        m_Winch.setWantedState(WinchStates.Holding);
    }
  }
    if(m_ArmPivot.getPosition()==PivotPositions.FrontL2){
      if(m_EndEffector.getCoralDetected()){
      m_EndEffector.setWantedState(EndEffectorState.L2Score);
      }
      if(!m_EndEffector.getCoralDetected()){
        m_EndEffector.setWantedState(EndEffectorState.Hold);
        m_ArmPivot.setPivotPosition(PivotPositions.Rest);
        m_Winch.setWantedState(WinchStates.Holding);
    }
  }
    if(m_ArmPivot.getPosition()==PivotPositions.FrontL3){
      if(m_EndEffector.getCoralDetected()){
      m_EndEffector.setWantedState(EndEffectorState.L3Score);
      }
      if(!m_EndEffector.getCoralDetected()){
        m_EndEffector.setWantedState(EndEffectorState.Hold);
        m_ArmPivot.setPivotPosition(PivotPositions.Rest);
        m_Winch.setWantedState(WinchStates.Holding);
    } 
  }
}
  public void setWantedState(states wantedState){
    this.wantedState = wantedState;
  }

  


}
