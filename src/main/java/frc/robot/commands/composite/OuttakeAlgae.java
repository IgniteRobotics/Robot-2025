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
import frc.robot.subsystems.algae.AlgaeCollector;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class OuttakeAlgae extends SequentialCommandGroup {
  Elevator m_elevator;
  AlgaeCollector m_collector;
  public OuttakeAlgae(Elevator elevator, AlgaeCollector collector) {
    m_elevator = elevator;
    m_collector = collector;
    addCommands(new ToSetpoint(m_elevator, ElevatorConstants.FLOOR.GROUND.position),
    new RunCommand(() -> m_collector.setWristPosition(-0.12)).until(() -> m_collector.isWristAtPosition()),
    new RunCommand(() -> m_collector.outtakeAlgae()));
  }
}
