// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.endeffector;

import frc.robot.RobotState;
import frc.robot.subsystems.EndEffector.EndEffector;
import edu.wpi.first.wpilibj2.command.Command;

public class AutoIngestCoral extends Command {
  private EndEffector m_endEffector;
  boolean seesCoralEnter;

  
  public AutoIngestCoral(EndEffector endEffector) {
    m_endEffector = endEffector;
    addRequirements(m_endEffector);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    seesCoralEnter = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_endEffector.seesCoralEnter()) seesCoralEnter = true;

    if(seesCoralEnter){
      m_endEffector.intakeCoral();
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_endEffector.stopCoralMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_endEffector.coralPreped();
  }
}
