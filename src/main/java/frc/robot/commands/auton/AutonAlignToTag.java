// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auton;

import java.util.Optional;
import java.util.function.Supplier;

import org.photonvision.PhotonCamera;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Preferences;
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
public class AutonAlignToTag extends Command {
  private final CommandSwerveDrivetrain m_drive;
  private final Supplier<Pose2d> m_targetPoseSupplier;
  private Pose2d m_initialBotPoseFieldRelative = null;
  private Pose2d m_targetPoseFieldRelative = null;
    
  PIDController rotationController;
  PIDController driveYController;
  PIDController driveXController;
  AprilTagFieldLayout aprilTags = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
  Constraints m_YConstraints;
  Constraints m_XConstraints;
  Constraints m_RotConstraints;

  private boolean doTranslation;
  
  private double m_distanceMeters;

  private int lockedTarget;

  private double m_rotation = 0;
  private double m_driveX = 0;
  private double m_driveY = 0;
  
  /** Creates a new AlignToTarget. */
  public AutonAlignToTag(CommandSwerveDrivetrain drive, Supplier<Pose2d> targetPoseSupplier){
    m_drive = drive;
    m_targetPoseSupplier = targetPoseSupplier;
    addRequirements(m_drive);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
    rotationController = new PIDController(Preferences.alignRotKP.get(), Preferences.alignRotKI.get(), Preferences.alignRotKD.get());
    rotationController.setTolerance(Preferences.rotationTolerancePreference.get());
    rotationController.enableContinuousInput(-180, 180);
    
    driveYController = new PIDController(Preferences.alignDriveYKP.get(), Preferences.alignDriveYKI.get(), Preferences.alignDriveYKD.get());
    driveYController.setTolerance(Preferences.yAlignTolerancePreference.get());
    driveYController.setIZone(Double.POSITIVE_INFINITY);

    driveXController = new PIDController(Preferences.alignDriveXKP.get(), Preferences.alignDriveXKI.get(), Preferences.alignDriveXKD.get());
    driveXController.setTolerance(Preferences.xAlignTolerancePreference.get());
    driveXController.setIZone(Double.POSITIVE_INFINITY);

    doTranslation = false;
    lockedTarget = -1;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    m_rotation = 0;
    m_driveX = 0;
    m_driveY = 0;

    if (m_targetPoseSupplier.get() != null) {
      m_targetPoseFieldRelative = m_targetPoseSupplier.get();
    }
    
    if (m_targetPoseFieldRelative != null) {
     
      //current field relative pose
      Pose2d currentPose = m_drive.getPose();

      SmartDashboard.putNumber("Alignment/Pose/TargetX", m_targetPoseFieldRelative.getTranslation().getX());
      SmartDashboard.putNumber("Alignment/Pose/TargetY", m_targetPoseFieldRelative.getTranslation().getY());
      SmartDashboard.putNumber("Alignment/Pose/TargetRot", m_targetPoseFieldRelative.getRotation().getDegrees());

      m_rotation = rotationController.calculate(m_drive.getYaw(), m_targetPoseFieldRelative.getRotation().getDegrees());
      SmartDashboard.putNumber("Alignment/Data/HeadingError", rotationController.getPositionError());
      SmartDashboard.putBoolean("Alignment/Data/atRotationSetpoint", rotationController.atSetpoint());


        
      m_driveY = driveYController.calculate(currentPose.getY(), m_targetPoseFieldRelative.getY());
      m_driveY = MathUtil.clamp(m_driveY, -2.5, 2.5);

      SmartDashboard.putNumber("Alignment/Data/YError", driveYController.getPositionError());
      SmartDashboard.putBoolean("Alignment/Data/atYSetpoint", driveYController.atSetpoint());
    

      m_driveX = driveXController.calculate(currentPose.getX(), m_targetPoseFieldRelative.getX());
      m_driveX = MathUtil.clamp(m_driveX, -2.5, 2.5);

      SmartDashboard.putNumber("Alignment/Data/DistanceError", driveXController.getPositionError());
      SmartDashboard.putBoolean("Alignment/Data/atDistanceSetpoint", driveXController.atSetpoint());
      
    }
    
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
