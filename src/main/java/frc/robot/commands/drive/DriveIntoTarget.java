// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import java.util.Optional;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Preferences;
import frc.robot.statemachines.AllianceState;
import frc.robot.statemachines.CoralState;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.drive.PhotonCameraWrapper;
import frc.robot.subsystems.drive.PhotonCameraWrapper.TargetInfo;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.epilogue.Logged;


@Logged
public class DriveIntoTarget extends Command {
  private final CommandSwerveDrivetrain m_drive;
  PhotonCameraWrapper m_pcw;
  PIDController driveXController;
  AprilTagFieldLayout aprilTags = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

  private int m_cameraId;
  
  private int targetIDs[] = {};


  private double m_distanceMeters;
  private double m_yawDegrees;
  
  /** Creates a new AlignToTarget. */
  public DriveIntoTarget(CommandSwerveDrivetrain drive,  PhotonCameraWrapper pcw){
    m_drive = drive;
    m_pcw = pcw;
    addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driveXController = new PIDController(Preferences.alignDriveXKP.get(), Preferences.alignDriveXKI.get(), Preferences.alignDriveXKD.get());
    driveXController.setTolerance(Preferences.xAlignTolerancePreference.get());
  
    targetIDs = AllianceState.getInstance().getReefTags();
    m_cameraId = CoralState.getInstance().pickCamera();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //Optional<TargetInfo> targeting = m_pcw.seekGeneralTargets(targetIDs, m_cameraId);
    
    Optional<TargetInfo> targeting = m_pcw.seekOuttakeTargets(targetIDs, m_cameraId);
    double driveX;

    if(targeting.isPresent()){
    
      m_distanceMeters = 0.347;
      driveX = driveXController.calculate(targeting.get().getDistance(), m_distanceMeters);
      SmartDashboard.putNumber("Alignment/Data/Distance", targeting.get().getDistance());
    
    }

    else{
      driveX = 0;
    }
  
    

    SmartDashboard.putNumber("Alignment/Power/driveX", driveX);
    
    m_drive.driveRobotCentric(driveX, 0, 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.driveRobotCentric(0, 0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return driveXController.atSetpoint();
  }
}
