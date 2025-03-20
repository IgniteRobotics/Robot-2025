// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.composite;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.PreferenceTypes.DoublePreference;
import frc.robot.commands.drive.ManualDriveAlignment;
import frc.robot.commands.corraler.OuttakeCommand;
import frc.robot.commands.elevator.ElevatorToCoralPreset;
import frc.robot.statemachines.CoralState.CoralTarget;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.coral.Corraler;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.drive.PhotonCameraWrapper;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SemiAutoScoreCoralGroup extends ParallelCommandGroup{

  private CommandSwerveDrivetrain m_swerveDrivetrain;
  private Elevator m_Elevator;
  private Corraler m_Corraler;
  private CoralTarget m_CoralTarget;
  private PhotonCameraWrapper m_PhotonCameraWrapper;
  private DoublePreference m_distancePreference;
  private DoubleSupplier m_DriveFwdBackSupplier;
  private DoubleSupplier m_DriveSideSupplier;
  private DoubleSupplier m_DriveRotSupplier;
  private BooleanSupplier m_raiseElevator;
  private BooleanSupplier m_releaseCoral;


  /** Creates a new CommandFactory. */
  public SemiAutoScoreCoralGroup(CommandSwerveDrivetrain swerveDrivetrain, Elevator elevator, Corraler corraler, 
      PhotonCameraWrapper photonCameraWrapper, DoublePreference distancePreference, DoubleSupplier driveFwdBackSupplier, DoubleSupplier driveSideSupplier, DoubleSupplier rotSupplier, BooleanSupplier raiseElevator, BooleanSupplier releaseCoral){
    m_swerveDrivetrain = swerveDrivetrain;
    m_Elevator = elevator;
    m_Corraler = corraler;
    m_PhotonCameraWrapper = photonCameraWrapper;
    m_distancePreference = distancePreference;
    m_DriveFwdBackSupplier = driveFwdBackSupplier;
    m_DriveSideSupplier = driveSideSupplier;
    m_DriveRotSupplier = rotSupplier;
    m_raiseElevator = raiseElevator;
    m_releaseCoral = releaseCoral;

    this.addCommands(createCommand());
  }

  public ParallelCommandGroup createCommand(){
    return createAlignCommand()
      .alongWith(new WaitUntilCommand(m_raiseElevator)
                  .andThen(new ElevatorToCoralPreset(m_Elevator)
                  .andThen(new WaitUntilCommand(m_releaseCoral)
                    .andThen(new OuttakeCommand(m_Corraler))))
      );
  }

  private Command createAlignCommand(){
    return new ManualDriveAlignment(m_swerveDrivetrain,
        m_DriveFwdBackSupplier, m_DriveSideSupplier, m_DriveRotSupplier);
  
  }

  // private Command createCoralCommand(){
  //  return 
  // }

  
 

}

