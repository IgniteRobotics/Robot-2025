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
import frc.robot.commands.drive.AlignToReefTags;
import frc.robot.statemachines.CoralState;
import frc.robot.statemachines.CoralState.CoralTarget;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.Elevator.ElevatorConstants;
import frc.robot.subsystems.coral.Corraler;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.drive.PhotonCameraWrapper;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoScoreCoralGroup extends ParallelCommandGroup{

  private CommandSwerveDrivetrain m_swerveDrivetrain;
  private Elevator m_Elevator;
  private Corraler m_Corraler;
  private CoralTarget m_CoralTarget;
  private PhotonCameraWrapper m_PhotonCameraWrapper;
  private DoublePreference m_distancePreference;
  private DoubleSupplier m_DriveFwdBackSupplier;
  private BooleanSupplier m_go;


  /** Creates a new CommandFactory. */
  public AutoScoreCoralGroup(CommandSwerveDrivetrain swerveDrivetrain, Elevator elevator, Corraler corraler, 
      PhotonCameraWrapper photonCameraWrapper, DoublePreference distancePreference, DoubleSupplier driveFwdBackSupplier, BooleanSupplier go){
    m_swerveDrivetrain = swerveDrivetrain;
    m_Elevator = elevator;
    m_Corraler = corraler;
    m_PhotonCameraWrapper = photonCameraWrapper;
    m_distancePreference = distancePreference;
    m_DriveFwdBackSupplier = driveFwdBackSupplier;
    m_go = go;

    this.addCommands(createCommand());
  }

  public ParallelCommandGroup createCommand(){
    return createAlignCommand()
      .alongWith(new WaitUntilCommand(m_go)
                  .andThen(new ScoreCoral(m_Elevator, m_Corraler, CoralState.getInstance().getCoralHeight(), ElevatorConstants.FLOOR.GROUND.position)
                  ).finallyDo(() -> CoralState.getInstance().setCoralTarget(CoralTarget.NONE))
      );
  }

  private Command createAlignCommand(){
    return new AlignToReefTags(m_swerveDrivetrain, m_PhotonCameraWrapper,
        m_DriveFwdBackSupplier, null);
  
  }

  // private Command createCoralCommand(){
  //  return 
  // }

  
 

}

