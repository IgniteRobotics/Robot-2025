// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import java.util.Optional;
import java.util.function.DoubleSupplier;

import org.photonvision.PhotonCamera;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Preferences;
import frc.robot.generated.TunerConstants;
import frc.robot.statemachines.AllianceState;
import frc.robot.statemachines.CoralState;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.drive.PhotonCameraWrapper;
import frc.robot.subsystems.drive.PhotonCameraWrapper.TargetInfo;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.epilogue.Logged;


@Logged
public class AlignToReefTags extends Command {
  private final CommandSwerveDrivetrain m_drive;
  PhotonCameraWrapper m_pcw;
  PIDController rotationController;
  PIDController driveYController;
  PIDController driveXController;
  AprilTagFieldLayout aprilTags = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

  private boolean atRotationSetpoint;
  private boolean atDriveYSetpoint;
  private boolean atDriveXSetpoint;
  
  private int targetIDs[] = {};


  private final DoubleSupplier m_xInput;
  private final DoubleSupplier m_yInput;

  private double m_distanceMeters;

  private double m_rotation = 0;
  private double m_driveX = 0;
  private double m_driveY = 0;
  
  /** Creates a new AlignToTarget. */
  public AlignToReefTags(CommandSwerveDrivetrain drive,  PhotonCameraWrapper pcw, DoubleSupplier xInput, DoubleSupplier yInput){
    m_drive = drive;
    m_pcw = pcw;
    m_xInput = xInput;
    m_yInput = yInput;
    addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    rotationController = new PIDController(Preferences.alignRotKP.get(), 0, Preferences.alignRotKD.get());
    rotationController.setTolerance(Preferences.rotationTolerancePreference.get());
    rotationController.enableContinuousInput(-180, 180);
    
    driveYController = new PIDController(Preferences.alignDriveYKP.get(), 0, Preferences.alignDriveYKD.get());
    driveYController.setTolerance(Preferences.yAlignTolerancePreference.get());

    driveXController = new PIDController(Preferences.alignDriveXKP.get(), 0, Preferences.alignDriveXKD.get());
    driveXController.setTolerance(Preferences.xAlignTolerancePreference.get());

    targetIDs = AllianceState.getInstance().getReefTags();

    atRotationSetpoint = false;
    atDriveYSetpoint = false;
    atDriveXSetpoint = false;

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //Optional<TargetInfo> targeting = m_pcw.seekGeneralTargets(targetIDs, m_cameraId);

    PhotonCamera camera = CoralState.getInstance().pickReefCamera();
    
    Optional<TargetInfo> targeting = m_pcw.seekTargets(targetIDs, camera);
    m_drive.lockToCamera(camera);

    m_rotation = 0;
    m_driveX = 0;
    m_driveY = 0;

    if(targeting.isPresent()){

      if(!atRotationSetpoint){
        double targetHeading = Math.toDegrees(aprilTags.getTagPose(targeting.get().getTagId()).get().getRotation().getZ());
        m_rotation = rotationController.calculate(m_drive.getYaw(), targetHeading);
        SmartDashboard.putNumber("Alignment/Data/Heading", targetHeading);

        atRotationSetpoint = rotationController.atSetpoint();
      }

      else if(!atDriveYSetpoint){
        m_driveY = -driveYController.calculate(targeting.get().getYaw(), CoralState.getInstance().getYCoralAlignment(targeting.get().getDistance()));
        SmartDashboard.putNumber("Alignment/Data/Yaw", targeting.get().getYaw());

        atDriveYSetpoint = driveYController.atSetpoint();
      }

      else if(!atDriveXSetpoint){
        m_distanceMeters = 0.347;
        m_driveX = driveXController.calculate(targeting.get().getDistance(), m_distanceMeters);
        SmartDashboard.putNumber("Alignment/Data/Distance", targeting.get().getDistance());

        atDriveXSetpoint = driveXController.atSetpoint();
      }

    }

    // else{
    //     //m_rotation = rotationController.calculate(m_drive.getYaw(), AllianceState.getInstance().getHeadingToReef(m_drive.getPose()));
    // }

    if(atDriveXSetpoint && atDriveYSetpoint && atRotationSetpoint){
      m_driveX = -Preferences.reefPushAgainstPreference.getValue();
    }
  
    //override with joystick input if present
    if(m_xInput != null && Math.abs(m_xInput.getAsDouble()) > TunerConstants.DEADBAND_FACTOR){
      m_driveX = 1.25*m_xInput.getAsDouble();
    }

    if(m_yInput != null && Math.abs(m_yInput.getAsDouble()) > TunerConstants.DEADBAND_FACTOR){
      m_driveY = 1.25*m_yInput.getAsDouble();
    }

    SmartDashboard.putNumber("Alignment/Power/rotation", m_rotation);
    SmartDashboard.putNumber("Alignment/Power/driveX", m_driveX);
    SmartDashboard.putNumber("Alignment/Power/driveY", m_driveY);
    
    m_drive.driveRobotCentric(m_driveX, m_driveY, m_rotation);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.unlockCameras();
    m_drive.driveRobotCentric(0, 0,0);
  }


  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
    //return rotationController.atSetpoint() && driveXController.atSetpoint() && driveYController.atSetpoint();
  }
}
