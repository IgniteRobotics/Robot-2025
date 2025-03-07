// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.elevator;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.statemachines.AlgaeState;
import frc.robot.statemachines.CoralState;
import frc.robot.statemachines.AlgaeState.AlgaeTarget;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.Elevator.ElevatorConstants;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ElevatorToAlgaePreset extends Command {
  Elevator m_elevator;
  AlgaeState m_algaeState;
  private double m_targetPosition = 0;
  /** Creates a new ElevatorToPreset. */
  public ElevatorToAlgaePreset(Elevator elevator) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_elevator = elevator;
    addRequirements(m_elevator);
    m_algaeState = AlgaeState.getInstance();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_algaeState.getAlgaeTarget() != AlgaeTarget.NONE) {
      m_targetPosition = m_algaeState.getAlgaeHeight();
      if (m_targetPosition != m_elevator.getTargetPosition()){
        m_elevator.setPositionRevolutions(m_targetPosition);
      }
    }
   
  
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_elevator.stopMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_elevator.atSetpoint();
  }
}
