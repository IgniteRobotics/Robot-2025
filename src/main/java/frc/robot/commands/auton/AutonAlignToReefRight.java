// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auton;

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
public class AutonAlignToReefRight extends Command {
  private final CommandSwerveDrivetrain m_drive;
  PhotonCameraWrapper m_pcw;
  PIDController rotationController;
  PIDController driveYController;
  PIDController driveXController;
  AprilTagFieldLayout aprilTags = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

  private boolean atRotationSetpoint;
  private boolean atDriveYSetpoint;
  private boolean atDriveXSetpoint;

  private int m_cameraId;
  
  private int targetIDs[] = {};


  private double m_distanceMeters;
  private double m_yawDegrees;

  private double m_rotation = 0;
  private double m_driveX = 0;
  private double m_driveY = 0;
  
  /** Creates a new AlignToTarget. */
  public AutonAlignToReefRight(CommandSwerveDrivetrain drive,  PhotonCameraWrapper pcw){
    m_drive = drive;
    m_pcw = pcw;
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
    
    Optional<TargetInfo> targeting = m_pcw.seekTargets(targetIDs, CameraConstants.photonCameraOuttakeRight);

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
        m_driveY = -driveYController.calculate(targeting.get().getYaw(), CameraConstants.getCorallYawOffsetDegreesRight(targeting.get().getDistance()));
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

    else{
        //m_rotation = rotationController.calculate(m_drive.getYaw(), AllianceState.getInstance().getHeadingToReef(m_drive.getPose()));
    }

    SmartDashboard.putNumber("Alignment/Power/rotation", m_rotation);
    SmartDashboard.putNumber("Alignment/Power/driveX", m_driveX);
    SmartDashboard.putNumber("Alignment/Power/driveY", m_driveY);
    
    m_drive.driveRobotCentric(m_driveX, m_driveY, m_rotation);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.driveRobotCentric(0, 0,0);
  }


  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return rotationController.atSetpoint() && driveXController.atSetpoint() && driveYController.atSetpoint();
  }
}
