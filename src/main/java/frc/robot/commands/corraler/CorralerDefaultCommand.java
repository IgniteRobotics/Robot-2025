// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.corraler;

import frc.robot.subsystems.coral.Corraler;
import edu.wpi.first.wpilibj2.command.Command;

public class CorralerDefaultCommand extends Command {
  private Corraler m_coraller;
  private boolean m_coralEntering;

  
  public CorralerDefaultCommand(Corraler corraler) {
    m_coraller = corraler;
    addRequirements(m_coraller);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_coralEntering = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if(m_coraller.seesCoralEnter()){
      m_coralEntering = true;
    }
    
    else if(!m_coraller.seesCoralEnter() && m_coraller.coralPreped()){
      m_coralEntering = false;
    }


    if(m_coralEntering){
      m_coraller.intakeCoral();
    }
    else m_coraller.stopCoralMotor();

    //What?
    /* 
    if (m_coraller.seesAlgae()){
      m_endEffector.holdAlgae();
      m_endEffector.setWristPosition(Preferences.endEffectorStowPositionWithAlgae);
    } else {
      m_endEffector.setWristPosition(Preferences.endEffectorStowPositionNoAlgae);
    }
      */


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_coraller.stopCoralMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
