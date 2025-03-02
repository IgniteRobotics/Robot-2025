// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.composite;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.elevator.ToSetpoint;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.Elevator.ElevatorConstants;
import frc.robot.subsystems.EndEffector.EndEffector;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class OuttakeAlgae extends SequentialCommandGroup {
  Elevator m_elevator;
  EndEffector m_effector;
  public OuttakeAlgae(Elevator elevator, EndEffector effector) {
    m_elevator = elevator;
    m_effector = effector;
    addCommands(new ToSetpoint(m_elevator, ElevatorConstants.FLOOR.GROUND.position),
    new RunCommand(() -> m_effector.setWristPosition(0)).until(() -> m_effector.isWristAtPosition()),
    new RunCommand(() -> m_effector.outtakeAlgae()).withTimeout(.5));
  }
}
