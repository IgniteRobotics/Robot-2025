// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auton;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.corraler.OuttakeCommand;
import frc.robot.commands.elevator.ElevatorToCoralPreset;
import frc.robot.commands.elevator.ToSetpoint;
import frc.robot.statemachines.CoralState;
import frc.robot.statemachines.CoralState.CoralTarget;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.Elevator.ElevatorConstants;
import frc.robot.subsystems.coral.Corraler;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.drive.PhotonCameraWrapper;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutonScoreCoralGroup extends ParallelCommandGroup{

  private CommandSwerveDrivetrain m_swerveDrivetrain;
  private Elevator m_Elevator;
  private Corraler m_Corraler;
  private PhotonCameraWrapper m_PhotonCameraWrapper;
  private DoubleSupplier m_DriveFwdBackSupplier;
  private DoubleSupplier m_DriveSideSupplier;
  private CoralTarget m_Position;

  /** Creates a new CommandFactory. */
  public AutonScoreCoralGroup(CommandSwerveDrivetrain swerveDrivetrain, Elevator elevator, Corraler corraler, 
      PhotonCameraWrapper photonCameraWrapper, CoralTarget coralPosition){
    m_swerveDrivetrain = swerveDrivetrain;
    m_Elevator = elevator;
    m_Corraler = corraler;
    m_PhotonCameraWrapper = photonCameraWrapper;
    m_Position = coralPosition;
    
    this.addCommands(createCommand());
  }

  public SequentialCommandGroup createCommand(){
    return new InstantCommand(() -> CoralState.getInstance().setCoralTarget(m_Position))
        .andThen(new AutonAlignToReefTag(m_swerveDrivetrain, m_PhotonCameraWrapper))
        .andThen(new ElevatorToCoralPreset(m_Elevator))
        .andThen(new OuttakeCommand(m_Corraler))
        .andThen(new InstantCommand(() -> CoralState.getInstance().setCoralTarget(CoralTarget.NONE)))
        .andThen(new ToSetpoint(m_Elevator, ElevatorConstants.FLOOR.GROUND.position))
      ;
  }
}

