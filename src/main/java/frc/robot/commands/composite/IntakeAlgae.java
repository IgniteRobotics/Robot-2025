// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.composite;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.commands.drive.AlignIntakeSide;
import frc.robot.commands.elevator.ToSetpoint;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.algae.AlgaeCollector;
import frc.robot.subsystems.drive.CameraConstants;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class IntakeAlgae extends ParallelCommandGroup{
  CommandSwerveDrivetrain m_drive;
  Elevator m_elevator;
  AlgaeCollector m_collector;
  int m_targets[];
  int m_cameraID = 0;
  double m_distance;
  double m_yawDegrees; 
  DoubleSupplier m_xInput;
  DoubleSupplier m_yInput;
  double position;
  public IntakeAlgae(CommandSwerveDrivetrain drive, int targets[], double distance, DoubleSupplier xInput, DoubleSupplier yInput, Elevator elevator, AlgaeCollector collector, double height) {
    m_drive = drive;
    m_targets = targets;
    m_distance = distance;
    m_yawDegrees = CameraConstants.getAlgaeYawOffsetDegreesLeft(m_distance);
    m_xInput = xInput;
    m_yInput = yInput;
    m_elevator = elevator;
    m_collector = collector;
    position = height;
    addCommands(new ToSetpoint(m_elevator, position), new RunCommand(() -> collector.setToIntakePosition()), new AlignIntakeSide(drive, drive.m_photonCameraWrapper, targets, m_distance, m_yawDegrees, xInput, yInput));
  }
}
