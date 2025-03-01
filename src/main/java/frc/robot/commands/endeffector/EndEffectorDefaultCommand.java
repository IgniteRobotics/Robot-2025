// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.endeffector;

import frc.robot.RobotState;
import frc.robot.subsystems.EndEffector.EndEffector;
import frc.robot.subsystems.EndEffector.EndEffectorConstants;

import frc.robot.Preferences;

import edu.wpi.first.wpilibj2.command.Command;

public class EndEffectorDefaultCommand extends Command {
  private EndEffector m_endEffector;
  private boolean m_coralEntering;

  
  public EndEffectorDefaultCommand(EndEffector endEffector) {
    m_endEffector = endEffector;
    addRequirements(m_endEffector);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_coralEntering = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if(m_endEffector.seesCoralEnter()){
      m_coralEntering = true;
    }
    
    else if(!m_endEffector.seesCoralEnter() && m_endEffector.coralPreped()){
      m_coralEntering = false;
    }


    if(m_coralEntering){
      m_endEffector.intakeCoral();
    }
    else m_endEffector.stopCoralMotor();


    if (m_endEffector.seesAlgae()){
      m_endEffector.holdAlgae();
      m_endEffector.setWristPosition(Preferences.endEffectorStowPositionWithAlgae.get());
    } else {
      m_endEffector.setWristPosition(Preferences.endEffectorStowPositionNoAlgae.get());
    }

    m_endEffector.setWristPosition(null);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_endEffector.stopCoralMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
