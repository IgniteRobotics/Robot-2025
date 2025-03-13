// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import java.util.Optional;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Preferences;
import frc.robot.PreferenceTypes.DoublePreference;
import frc.robot.generated.TunerConstants;
import frc.robot.statemachines.AllianceState;
import frc.robot.statemachines.CoralState;
import frc.robot.subsystems.drive.CameraConstants;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.drive.PhotonCameraWrapper;
import frc.robot.subsystems.drive.PhotonCameraWrapper.TargetInfo;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.epilogue.Logged;


@Logged
public class AlignSideToSide extends Command {
  private final CommandSwerveDrivetrain m_drive;
  PhotonCameraWrapper m_pcw;
  PIDController rotationController;
  PIDController driveYController;
  PIDController driveXController;
  AprilTagFieldLayout aprilTags = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

  private int m_cameraId;
  
  private int targetIDs[] = {};


  private double m_distanceMeters;
  private double m_yawDegrees;
  
  /** Creates a new AlignToTarget. */
  public AlignSideToSide(CommandSwerveDrivetrain drive,  PhotonCameraWrapper pcw){
    m_drive = drive;
    m_pcw = pcw;
    addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driveYController = new PIDController(Preferences.alignDriveYKP.get(), 0, Preferences.alignDriveYKD.get());
    driveYController.setTolerance(Preferences.yAlignTolerancePreference.get());
  
    targetIDs = AllianceState.getInstance().getReefTags();
    m_cameraId = CoralState.getInstance().pickCamera();
    m_distanceMeters = Preferences.coralXDriveOffset.get();
    m_yawDegrees = CoralState.getInstance().getYCoralAlignment(m_distanceMeters);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //Optional<TargetInfo> targeting = m_pcw.seekGeneralTargets(targetIDs, m_cameraId);
    
    Optional<TargetInfo> targeting = m_pcw.seekTargets(targetIDs, m_cameraId);
    double driveY;

    if(targeting.isPresent()){
    
      
      driveY = -driveYController.calculate(targeting.get().getYaw(), 0);
    }

    else{
      driveY = 0;
    }
  
    

    SmartDashboard.putNumber("Alignment/Power/driveY", driveY);
    
    m_drive.driveRobotCentric(0, driveY, 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.driveRobotCentric(0, 0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return driveYController.atSetpoint();
  }
}
