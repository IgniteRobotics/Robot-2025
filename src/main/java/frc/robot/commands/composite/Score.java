// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.composite;

import javax.management.InstanceNotFoundException;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Preferences;
import frc.robot.commands.elevator.ToSetpoint;
import frc.robot.statemachines.AlgaeState;
import frc.robot.statemachines.CoralState;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.Elevator.ElevatorConstants;
import frc.robot.subsystems.algae.AlgaeCollector;
import frc.robot.subsystems.coral.Corraler;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;


public class Score extends SequentialCommandGroup {
  AlgaeState m_algaeState = AlgaeState.getInstance();
  CoralState m_coralState = CoralState.getInstance();
  public Score(Elevator elevator, 
                           Corraler corraler, 
                           AlgaeCollector collector,
                           CommandSwerveDrivetrain drive) {
    
    //ready to score coral
    if (m_coralState.hasCoral() && m_coralState.getCoralHeight() != 0){
      addCommands(new ScoreCoral(elevator, corraler, m_coralState.getCoralHeight(), ElevatorConstants.FLOOR.GROUND.position));
    //ready to score algae
    } else if (m_algaeState.hasAlgae() && m_algaeState.getAlgaeHeight() != 0){
      addCommands(
        new ParallelCommandGroup(
          new ToSetpoint(elevator, m_algaeState.getAlgaeHeight()),
          new RunCommand(() -> collector.setWristPosition(m_algaeState.getAlgaeWristPosition()))
            .withName("SetWristPosition")
            .until(() -> collector.isWristAtPosition())
        ),
        new RunCommand(()-> collector.outtakeAlgae())
          .withName("EjectAlgae")
          .withTimeout(.50),
        new ParallelCommandGroup(
          new ToSetpoint(elevator, ElevatorConstants.FLOOR.GROUND.position),
          new RunCommand(() -> collector.setWristPosition(m_algaeState.getAlgaeWristPosition()))
            .withName("StowWrist")
            .until(() -> collector.isWristAtPosition())
        )
      );
    //ready to intake algae from reef
    } else if (!m_coralState.hasCoral() && !m_algaeState.hasAlgae() && m_algaeState.getAlgaeHeight() !=0){
      addCommands(
        new ParallelCommandGroup(
          new ToSetpoint(elevator, m_algaeState.getAlgaeHeight()),
          new RunCommand(() -> collector.setWristPosition(m_algaeState.getAlgaeWristPosition()))
            .withName("SetWristPosition")
            .until(() -> collector.isWristAtPosition())
        ),
        new RunCommand(()-> collector.intakeAlgae())
          .withName("IntakeAlgae")
          .until(() -> m_algaeState.hasAlgae()),
        new ParallelCommandGroup(
          new ToSetpoint(elevator, m_algaeState.getAlgaeHeight()),
          new RunCommand(() -> collector.setWristPosition(m_algaeState.getAlgaeWristPosition()))
            .withName("StowWrist")
            .until(() -> collector.isWristAtPosition())
        )
      ); 

    } else {
      //do nothing
      addCommands(new InstantCommand(() -> {}));
    }
  }

}
