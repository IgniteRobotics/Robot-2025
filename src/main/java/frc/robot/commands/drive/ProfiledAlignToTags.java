// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import java.util.Optional;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.photonvision.PhotonCamera;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;

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
public class ProfiledAlignToTags extends Command {
  private final CommandSwerveDrivetrain m_drive;
  PhotonCameraWrapper m_pcw;
    PIDController rotationController;

  ProfiledPIDController driveYController;
  ProfiledPIDController driveXController;
  AprilTagFieldLayout aprilTags = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
  Constraints m_YConstraints;
  Constraints m_XConstraints;
  Constraints m_RotConstraints;

  private boolean doTranslation;

  private boolean driverOverrideY;
  private boolean driverOverrideX;
  
  private Supplier<int[]> m_idSupplier;
  private Supplier<PhotonCamera> m_cameraSupplier;

  private final DoubleSupplier m_xInput;
  private final DoubleSupplier m_yInput;

  private double m_distanceMeters;

  private double m_rotation = 0;
  private double m_driveX = 0;
  private double m_driveY = 0;
  
  /** Creates a new AlignToTarget. */
  public ProfiledAlignToTags(CommandSwerveDrivetrain drive,  PhotonCameraWrapper pcw, Supplier<int[]> idSupplier, Supplier<PhotonCamera> cameraSupplier, DoubleSupplier xInput, DoubleSupplier yInput){
    m_drive = drive;
    m_pcw = pcw;
    m_idSupplier = idSupplier;
    m_cameraSupplier = cameraSupplier;
    m_xInput = xInput;
    m_yInput = yInput;
    addRequirements(m_drive);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
    rotationController = new PIDController(Preferences.profiledAlignRotKP.get(), Preferences.profiledAlignRotKI.get(), Preferences.profiledAlignRotKD.get());
    rotationController.setTolerance(Preferences.rotationTolerancePreference.get());
    rotationController.enableContinuousInput(-180, 180);
    
    m_YConstraints = new Constraints(Preferences.profiledDriveYMaxVel.get(), Preferences.profiledDriveYMaxAcc.get());
    driveYController = new ProfiledPIDController(Preferences.profiledDriveYKP.get(), Preferences.profiledDriveYKI.get(), Preferences.profiledDriveYKD.get(), m_YConstraints);
    driveYController.setTolerance(Preferences.yAlignTolerancePreference.get());
    driveYController.setIZone(Double.POSITIVE_INFINITY);

    m_XConstraints = new Constraints(Preferences.profiledDriveXMaxVel.get(), Preferences.profiledDriveXMaxAcc.get());
    driveXController = new ProfiledPIDController(Preferences.profiledDriveXKP.get(), Preferences.profiledDriveXKI.get(), Preferences.profiledDriveXKD.get(), m_XConstraints);
    driveXController.setTolerance(Preferences.xAlignTolerancePreference.get());
    driveXController.setIZone(Double.POSITIVE_INFINITY);

    doTranslation = false;

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    Optional<TargetInfo> targeting = m_pcw.seekTargets(m_idSupplier.get(), m_cameraSupplier.get());

    m_rotation = 0;
    m_driveX = 0;
    m_driveY = 0;

    if(targeting.isPresent()){

      SmartDashboard.putNumber("Alignment/Data/TargetID", targeting.get().getTagId());
      SmartDashboard.putNumber("Alignment/Data/TargetX", targeting.get().getTransform3d().getX());
      SmartDashboard.putNumber("Alignment/Data/TargetY", targeting.get().getTransform3d().getY());

      
      double targetHeading = Math.toDegrees(aprilTags.getTagPose(targeting.get().getTagId()).get().getRotation().getZ());
      m_rotation = rotationController.calculate(m_drive.getYaw(), targetHeading);
      SmartDashboard.putNumber("Alignment/Data/Heading", targetHeading);
      SmartDashboard.putNumber("Alignment/Data/HeadingError", rotationController.getPositionError());
      SmartDashboard.putNumber("Alignment/Data/HeadingAccumulatedError", rotationController.getAccumulatedError());
      SmartDashboard.putBoolean("Alignment/Data/atRotationSetpoint", rotationController.atSetpoint());

      if(rotationController.atSetpoint()){
        doTranslation = true;
      }

      if(doTranslation){
      
        if(!driverOverrideY){
          m_driveY = driveYController.calculate(targeting.get().getTransform3d().getY(), 0);
          m_driveY = MathUtil.clamp(m_driveY, -2, 2);
          SmartDashboard.putNumber("Alignment/Data/YError", driveYController.getPositionError());
          SmartDashboard.putNumber("Alignment/Data/YAccumulatedError", driveYController.getAccumulatedError());

          SmartDashboard.putBoolean("Alignment/Data/atYSetpoint", driveYController.atSetpoint());
        }
      
        if(!driverOverrideX){
          m_distanceMeters = 0.347;
          m_driveX = driveXController.calculate(targeting.get().getTransform3d().getX(), m_distanceMeters);
          //if Y alignment is still running, scale X alignment power to curve in.
          // if (!atDriveYGoal) {
          //   m_driveX = MathUtil.clamp(m_driveX, -m_driveX*0.5, m_driveX*0.5);
          // }
          SmartDashboard.putNumber("Alignment/Data/DistanceError", driveXController.getPositionError());
          SmartDashboard.putNumber("Alignment/Data/DistanceAccumulatedError", driveXController.getAccumulatedError());
          SmartDashboard.putBoolean("Alignment/Data/atXSetpoint", driveXController.atSetpoint());
        }
      }
    }
  
    //override with joystick input if present
    if(m_xInput != null && Math.abs(m_xInput.getAsDouble()) > TunerConstants.DEADBAND_FACTOR){
      driverOverrideX = true;
      m_driveX = Preferences.xySlowLimitPreference.getValue()*m_xInput.getAsDouble();
    } else if (driverOverrideX && (m_xInput == null || Math.abs(m_xInput.getAsDouble()) <= TunerConstants.DEADBAND_FACTOR)){
      m_driveX = 0;
    }

    if(m_yInput != null && Math.abs(m_yInput.getAsDouble()) > TunerConstants.DEADBAND_FACTOR){
      driverOverrideY = true;
      m_driveY = Preferences.xySlowLimitPreference.getValue()*m_yInput.getAsDouble();
    } else if (driverOverrideY && (m_xInput == null || Math.abs(m_yInput.getAsDouble()) <= TunerConstants.DEADBAND_FACTOR)){
      m_driveY = 0;
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
    return false;
    //return rotationController.atSetpoint() && driveXController.atSetpoint() && driveYController.atSetpoint();
  }
}
