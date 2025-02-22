// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator.Elevator;

public class ToSetpoint extends Command {
  Elevator m_elevator;
  double setpoint;
  public ToSetpoint(Elevator elevator, double position) {
    m_elevator = elevator;
    setpoint = position;
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    m_elevator.setPositionRevolutions(setpoint);
  }

  @Override
  public void end(boolean interrupted) {}

 
  @Override
  public boolean isFinished() {
    return m_elevator.atSetpoint();
  }
}
