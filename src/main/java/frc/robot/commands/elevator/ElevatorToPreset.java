// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.elevator;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.statemachines.RobotState;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.Elevator.ElevatorConstants;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ElevatorToPreset extends Command {
  Elevator m_elevator;
  RobotState m_robotState;
  private double m_targetPosition = 0;
  /** Creates a new ElevatorToPreset. */
  public ElevatorToPreset(Elevator elevator) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_elevator = elevator;
    addRequirements(m_elevator);
    m_robotState = RobotState.getInstance();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_robotState.getCoralTarget() != RobotState.CoralTarget.NONE) {
      m_targetPosition = m_robotState.getCoralHeight();
    } else if (m_robotState.getAlgaeTarget() != RobotState.AlgaeTarget.NONE) {
      m_targetPosition = m_robotState.getAlgaeHeight();
    }
    if (m_targetPosition != m_elevator.getPosition()) {
      m_elevator.setPositionRevolutions(m_targetPosition);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_elevator.atSetpoint();
  }
}
