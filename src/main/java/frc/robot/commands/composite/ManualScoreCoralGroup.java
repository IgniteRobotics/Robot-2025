
    // Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.composite;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
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
public class ManualScoreCoralGroup extends SequentialCommandGroup{

  private Elevator m_Elevator;
  private Corraler m_Corraler;
  private BooleanSupplier m_raiseElevator;
  private BooleanSupplier m_releaseCoral;


  /** Creates a new CommandFactory. */
  public ManualScoreCoralGroup(Elevator elevator, Corraler corraler,  BooleanSupplier raiseElevator, BooleanSupplier releaseCoral){
    m_Elevator = elevator;
    m_Corraler = corraler;
    m_raiseElevator = raiseElevator;
    m_releaseCoral = releaseCoral;

    this.addCommands(createCommand());
  }

  public Command createCommand(){
    return new WaitUntilCommand(m_raiseElevator)
                  .andThen(new ElevatorToCoralPreset(m_Elevator))
                  .andThen(new WaitUntilCommand(m_releaseCoral))
                            .andThen(new OuttakeCommand(m_Corraler))
                            .andThen(new InstantCommand(() -> CoralState.getInstance().setCoralTarget(CoralTarget.NONE)));
  }
}

