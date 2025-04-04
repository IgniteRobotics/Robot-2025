// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.composite;

import java.util.Map;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.PreferenceTypes.DoublePreference;
import frc.robot.commands.drive.DriveToPoseNoProfile;
import frc.robot.commands.drive.ManualDriveAlignment;
import frc.robot.commands.drive.ProfiledAlignToTags;
import frc.robot.commands.corraler.OuttakeCommand;
import frc.robot.commands.elevator.ElevatorToCoralPreset;
import frc.robot.commands.vision.FindReefTarget;
import frc.robot.statemachines.AllianceState;
import frc.robot.statemachines.CoralState;
import frc.robot.statemachines.DriveState;
import frc.robot.statemachines.CoralState.CoralTarget;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.coral.Corraler;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.drive.PhotonCameraWrapper;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoScoreCoralGroup extends ParallelCommandGroup{

  private CommandSwerveDrivetrain m_swerveDrivetrain;
  private Elevator m_Elevator;
  private Corraler m_Corraler;
  private PhotonCameraWrapper m_PhotonCameraWrapper;
  private DoubleSupplier m_DriveFwdBackSupplier;
  private DoubleSupplier m_DriveSideSupplier;
  private BooleanSupplier m_raiseElevator;
  private BooleanSupplier m_releaseCoral;
  private BooleanSupplier m_manualDrive;

  private enum CommandSelector{
    MANUAL_DRIVE,
    AUTO_DRIVE
  }

  private CommandSelector select(){
    if (m_manualDrive.getAsBoolean()){
      return CommandSelector.MANUAL_DRIVE;
    } else {
      return CommandSelector.AUTO_DRIVE;
    }

  }


  /** Creates a new CommandFactory. */
  public AutoScoreCoralGroup(CommandSwerveDrivetrain swerveDrivetrain, Elevator elevator, Corraler corraler, 
      PhotonCameraWrapper photonCameraWrapper, DoubleSupplier driveFwdBackSupplier, DoubleSupplier driveSideSupplier, BooleanSupplier raiseElevator, BooleanSupplier releaseCoral, BooleanSupplier manualDrive){
    m_swerveDrivetrain = swerveDrivetrain;
    m_Elevator = elevator;
    m_Corraler = corraler;
    m_PhotonCameraWrapper = photonCameraWrapper;
    m_DriveFwdBackSupplier = driveFwdBackSupplier;
    m_DriveSideSupplier = driveSideSupplier;
    m_raiseElevator = raiseElevator;
    m_releaseCoral = releaseCoral;
    m_manualDrive = manualDrive;

    this.addCommands(createCommand());
  }

  private final Command m_selectedDriveCommand = 
    new SelectCommand<>(
      Map.ofEntries(
        Map.entry(CommandSelector.AUTO_DRIVE, new DriveToPoseNoProfile(m_swerveDrivetrain, () -> DriveState.getInstance().getTargetPose2d(), m_DriveFwdBackSupplier, m_DriveSideSupplier, true)
          .alongWith(new FindReefTarget(m_PhotonCameraWrapper, () -> AllianceState.getInstance().getReefTags(), () -> CoralState.getInstance().pickReefCamera()))),
        Map.entry(CommandSelector.MANUAL_DRIVE, new ManualDriveAlignment(m_swerveDrivetrain, m_DriveFwdBackSupplier, m_DriveSideSupplier, () -> 0.0))
      ),
      this::select
    );

  public ParallelCommandGroup createCommand(){
    return m_selectedDriveCommand
      .alongWith(new WaitUntilCommand(m_raiseElevator)
                  .andThen(new ElevatorToCoralPreset(m_Elevator)
                  .andThen(new WaitUntilCommand(m_releaseCoral)
                            .andThen(new OuttakeCommand(m_Corraler))
                            .andThen(new InstantCommand(() -> CoralState.getInstance().setCoralTarget(CoralTarget.NONE)))))
      );
  }
}
