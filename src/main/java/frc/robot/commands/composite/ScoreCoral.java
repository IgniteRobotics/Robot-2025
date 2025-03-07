// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.composite;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Preferences;
import frc.robot.PreferenceTypes.DoublePreference;
import frc.robot.commands.corraler.OuttakeCommand;
import frc.robot.commands.elevator.ElevatorToPreset;
import frc.robot.commands.elevator.ToSetpoint;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.coral.Corraler;

public class ScoreCoral extends SequentialCommandGroup {
  Elevator m_elevator;
  Corraler m_corraler;
  double scoreHeight;
  double endHeight;
  public ScoreCoral(Elevator elevator, Corraler corraler, double score, double end) {
    m_elevator = elevator;
    m_corraler = corraler;
    scoreHeight = score;
    endHeight = end;
    addCommands(new ElevatorToPreset(elevator), new OuttakeCommand(m_corraler).withTimeout(1), new ToSetpoint(m_elevator, endHeight));
  }
}
