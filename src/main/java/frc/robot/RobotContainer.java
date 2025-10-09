// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Commands.SwerveDrive;
import frc.robot.Subsystems.Macros;
import frc.robot.Subsystems.ArmPivot.Pivot;
import frc.robot.Subsystems.ArmPivot.PivotIOSparkMax;
import frc.robot.Subsystems.ArmPivot.Pivot.PivotPositions;
import frc.robot.Subsystems.ArmWinch.ArmWinch;
import frc.robot.Subsystems.ArmWinch.ArmWinchIOSparkMax;
import frc.robot.Subsystems.ArmWinch.ArmWinch.WinchStates;
import frc.robot.Subsystems.EndEffector.EndEffector;
import frc.robot.Subsystems.EndEffector.EndEffectorIOSparkMax;
import frc.robot.Subsystems.EndEffector.EndEffector.EndEffectorState;
import frc.robot.Subsystems.Macros.states;
import frc.robot.Subsystems.SwerveDrive.Drivetrain;
import frc.robot.Subsystems.SwerveDrive.Gyro.*;
import frc.robot.Subsystems.SwerveDrive.Module.*;

public class RobotContainer {
  public static Drivetrain m_Drivetrain;
  public static final CommandXboxController driverController =
      new CommandXboxController(0);

  public static final GenericHID buttonBoard = 
  new GenericHID(1);
  public static final Pivot armPivot = new Pivot(new PivotIOSparkMax());
  public static final ArmWinch armWinch = new ArmWinch(new ArmWinchIOSparkMax());
  public static final EndEffector endEffector = new EndEffector(new EndEffectorIOSparkMax());
  public final JoystickButton resetHeading_Start =
      new JoystickButton(driverController.getHID(), XboxController.Button.kStart.value);
  public final SendableChooser<Command> autoChooser;
  public static Macros macros;

  public RobotContainer() {
    m_Drivetrain = new Drivetrain(new GyroPidgeonIO(), new ModuleIOSparkMax(1), new ModuleIOSparkMax(2), new ModuleIOSparkMax(3), new ModuleIOSparkMax(4));
    m_Drivetrain.setDefaultCommand(new SwerveDrive());
    macros = new Macros(armPivot, armWinch, endEffector);
    registerNamedCommands();
    configureBindings();
    autoChooser = AutoBuilder.buildAutoChooser();
  }

  private void configureBindings() {
    resetHeading_Start.onTrue(new InstantCommand(m_Drivetrain::zeroHeading, m_Drivetrain));
    driverController.rightTrigger().onTrue(new InstantCommand(()->macros.setWantedState(states.Score),macros));


    new Trigger(()->buttonBoard.getAxisType(2)>0.5).onTrue(new InstantCommand(()->macros.setWantedState(states.ResetArm),macros));
    new Trigger(()->buttonBoard.getRawButton(10)).onTrue(new InstantCommand(()->macros.setWantedState(states.GroundIntake),macros));
    new Trigger(()->buttonBoard.getRawButton(1)).onTrue(new InstantCommand(()->macros.setWantedState(states.L2),macros));
    new Trigger(()->buttonBoard.getRawButton(2)).onTrue(new InstantCommand(()->macros.setWantedState(states.L3),macros));
    new Trigger(()->buttonBoard.getRawButton(3)).onTrue(new InstantCommand(()->macros.setWantedState(states.L4),macros));
    new Trigger(()->buttonBoard.getRawButton(3)).onTrue(new InstantCommand(()->macros.setWantedState(states.L4Back),macros));//Should never have to go to L4Back Usually
    // driverController.povUp().whileTrue(new InstantCommand(()->armPivot.setPivotPosition(PivotPositions.ManualControl)));
    // driverController.povDown().whileTrue(new InstantCommand(()->armPivot.setPivotPosition(PivotPositions.ManualControl)));

    // driverController.y().whileTrue(new InstantCommand(()->armWinch.setWantedState(WinchStates.ManualControl),armWinch));
    // //driverController.a().onTrue();
    
    // driverController.b().onTrue(new InstantCommand(()->armWinch.setWantedState(WinchStates.TestPID),armWinch));
    // //driverController.a().whileTrue(new InstantCommand(()->endEffector.setWantedState(EndEffectorState.RollerVoltageControl),endEffector));
    // driverController.x().whileTrue(new InstantCommand(()->endEffector.setWantedState(EndEffectorState.RollerVoltageControl),endEffector));
    // driverController.povLeft().whileTrue(new InstantCommand(()->endEffector.setWantedState(EndEffectorState.RollerVoltageControl),endEffector));
    // driverController.povRight().whileTrue(new InstantCommand(()->endEffector.setWantedState(EndEffectorState.RollerVoltageControl),endEffector));
    //driverController.rightBumper().onTrue(new InstantCommand(()->armPivot.setPivotPosition(PivotPositions.GroundIntake),armPivot));
    // driverController.rightBumper().onTrue(new InstantCommand(()->armPivot.setPivotPosition(PivotPositions.GroundIntake)));
    // driverController.leftBumper().onTrue(new InstantCommand(()->macros.setWantedState(states.GroundIntake),macros));
    //driverController.rightTrigger().onTrue(new InstantCommand(()->endEffector.setWantedState(EndEffectorState.L4),endEffector));
    // driverController.leftTrigger().whileTrue(new InstantCommand(()->macros.setWantedState(states.L2),macros));
    //driverController.leftBumper().onTrue(new InstantCommand(()->endEffector.setWantedState(EndEffectorState.L4),endEffector));
    
    //driverController.leftBumper().onTrue(new InstantCommand(()->endEffector.setWantedState(EndEffectorState.GroundIntake),endEffector).until(()->endEffector.getCoralDetected()));
    // driverController.povUp().whileTrue(new InstantCommand(()->armPivot.setPivotPosition(PivotPositions.ManualControl),armPivot));
    
    // driverController.povDown().whileTrue(new InstantCommand(()->armPivot.setPivotPosition(PivotPositions.ManualControl),armPivot));
    // driverController.povRight().whileTrue(new InstantCommand(()->endEffector.setWantedState(EndEffectorState.RollerVoltageControl),endEffector));
    // driverController.povLeft().whileTrue(new InstantCommand(()->endEffector.setWantedState(EndEffectorState.RollerVoltageControl),endEffector));
    // driverController.povUp().whileTrue(new InstantCommand(()->armPivot.qualitisticRoutine(Direction.kForward),armPivot));
    // driverController.povRight().whileTrue(new InstantCommand(()->armPivot.qualitisticRoutine(Direction.kReverse),armPivot));
    // driverController.povLeft().whileTrue(new InstantCommand(()->armPivot.dynamicRoutine(Direction.kForward),armPivot));
    // driverController.povDown().whileTrue(new InstantCommand(()->armPivot.dynamicRoutine(Direction.kReverse),armPivot));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
  public void registerNamedCommands() {}
}
