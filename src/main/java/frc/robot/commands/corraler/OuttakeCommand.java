// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.corraler;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.PreferenceTypes.DoublePreference;
import frc.robot.subsystems.coral.Corraler;

public class OuttakeCommand extends Command {
  Corraler m_corraler;
  Timer delayTimer;
  
  
  public OuttakeCommand(Corraler effector) {
    m_corraler = effector;
    addRequirements(m_corraler);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    delayTimer = new Timer();
    delayTimer.stop();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_corraler.outtakeCoral();
    
    if(m_corraler.coralPreped() && !delayTimer.isRunning()){
      delayTimer.reset();
      delayTimer.start();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_corraler.stopCoralMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (delayTimer.hasElapsed(0.2));
  }
}
