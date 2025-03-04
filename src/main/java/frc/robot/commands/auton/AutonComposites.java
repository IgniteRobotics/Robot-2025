// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auton;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.elevator.ToSetpoint;
import frc.robot.statemachines.CoralState;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.Elevator.ElevatorConstants;
import frc.robot.subsystems.coral.Corraler;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.drive.PhotonCameraWrapper;

/** Add your docs here. */
public class AutonComposites {
    private final static CoralState m_coralState = CoralState.getInstance();
    
    public static Command ScoreLevel4ReefJ;

    public static Command ScoreLevel4ReefL;

    public static Command ScoreLevel4ReefK;

    public static Command IntakeCoralHP(CommandSwerveDrivetrain drive, PhotonCameraWrapper camera, Elevator elevator){
        return (new AutonAlignToHP(drive, camera).alongWith(new ToSetpoint(elevator , ElevatorConstants.FLOOR.HP.position))).andThen( new WaitUntilCommand(() -> m_coralState.hasCoral()));
    }
}
