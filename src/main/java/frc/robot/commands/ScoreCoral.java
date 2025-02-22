// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.PreferenceTypes.DoublePreference;
import frc.robot.commands.elevator.ToSetpoint;
import frc.robot.commands.endeffector.OuttakeCommand;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.EndEffector.EndEffector;

public class ScoreCoral extends SequentialCommandGroup {
  Elevator m_elevator;
  EndEffector m_effector;
  double scoreHeight;
  double endHeight;
  public ScoreCoral(Elevator elevator, EndEffector effector, double score, double end) {
    elevator = m_elevator;
    m_effector = effector;
    scoreHeight = score;
    endHeight = end;
    addCommands(new ToSetpoint(elevator, scoreHeight), new OuttakeCommand(effector).withTimeout(1), new ToSetpoint(elevator, endHeight));
  }
}
