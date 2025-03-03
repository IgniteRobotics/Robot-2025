// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.composite;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.elevator.ToSetpoint;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.algae.AlgaeCollector;
import frc.robot.subsystems.algae.AlgaeCollectorConstants;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class IntakeAlgae extends ParallelCommandGroup{
  Elevator m_elevator;
  AlgaeCollector m_collector;
  double position;
  public IntakeAlgae(Elevator elevator, AlgaeCollector collector, double height) {
    m_elevator = elevator;
    m_collector = collector;
    position = height;
    addCommands(new ToSetpoint(m_elevator, position), new RunCommand(() -> m_collector.setWristPosition(AlgaeCollectorConstants.ALGAE.REEF.angle)),
    new RunCommand(() -> m_collector.intakeAlgae()));
  }
}
