// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Preferences;
import frc.robot.generated.TunerConstants;
import frc.robot.statemachines.DriveState;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveToPose extends Command {
  /** Creates a new DriveToPoseRobotRelative. */
  private final CommandSwerveDrivetrain m_driveTrain;
  private final Supplier<Pose2d> m_targetPoseSupplier;
  private Pose2d m_initialBotPoseFieldRelative = null;
  private Pose2d m_targetPoseFieldRelative = null;

  PIDController m_rotationController;
  ProfiledPIDController m_driveYController;
  ProfiledPIDController m_driveXController;

  Constraints m_YConstraints;
  Constraints m_XConstraints;
  Constraints m_RotConstraints;

  boolean m_allowDriverOverride = false;
  boolean m_driverOverrideX = false;
  boolean m_driverOverrideY = false;

  private final DoubleSupplier m_xInput;
  private final DoubleSupplier m_yInput;

  public DriveToPose(CommandSwerveDrivetrain driveTrain, Supplier<Pose2d> targetPoseSupplier, DoubleSupplier xInput, DoubleSupplier yInput, boolean allowDriverOverride) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_driveTrain = driveTrain;
    m_targetPoseSupplier = targetPoseSupplier;
    m_xInput = xInput;
    m_yInput = yInput;
    m_allowDriverOverride = allowDriverOverride;
    addRequirements(m_driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_initialBotPoseFieldRelative = m_driveTrain.getPose();
  

    m_rotationController = new PIDController(Preferences.profiledAlignRotKP.get(), Preferences.profiledAlignRotKI.get(), Preferences.profiledAlignRotKD.get());
    m_rotationController.setTolerance(Preferences.rotationTolerancePreference.get());
    m_rotationController.enableContinuousInput(-180, 180);
    
    
    m_YConstraints = new Constraints(Preferences.profiledDriveYMaxVel.get(), Preferences.profiledDriveYMaxAcc.get());
    m_driveYController = new ProfiledPIDController(Preferences.profiledDriveYKP.get(), Preferences.profiledDriveYKI.get(), Preferences.profiledDriveYKD.get(), m_YConstraints);
    m_driveYController.setTolerance(Preferences.yAlignTolerancePreference.get());
    m_driveYController.reset(m_initialBotPoseFieldRelative.getY(), m_driveTrain.getState().Speeds.vxMetersPerSecond);
    m_driveYController.setIZone(Double.POSITIVE_INFINITY);

    m_XConstraints = new Constraints(Preferences.profiledDriveXMaxVel.get(), Preferences.profiledDriveXMaxAcc.get());
    m_driveXController = new ProfiledPIDController(Preferences.profiledDriveXKP.get(), Preferences.profiledDriveXKI.get(), Preferences.profiledDriveXKD.get(), m_XConstraints);
    m_driveXController.setTolerance(Preferences.xAlignTolerancePreference.get());
    m_driveXController.reset(m_initialBotPoseFieldRelative.getX(), m_driveTrain.getState().Speeds.vyMetersPerSecond);
    m_driveXController.setIZone(Double.POSITIVE_INFINITY);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double rotation = 0;
    double driveX = 0;
    double driveY = 0;

    if (m_targetPoseSupplier.get() != null) {
      m_targetPoseFieldRelative = m_targetPoseSupplier.get();
    }
    
    if (m_targetPoseFieldRelative == null) {
     
      //current field relative pose
      Pose2d currentPose = m_driveTrain.getPose();

      SmartDashboard.putNumber("Alignment/Pose/TargetX", m_targetPoseFieldRelative.getTranslation().getX());
      SmartDashboard.putNumber("Alignment/Pose/TargetY", m_targetPoseFieldRelative.getTranslation().getY());
      SmartDashboard.putNumber("Alignment/Pose/TargetRot", m_targetPoseFieldRelative.getRotation().getDegrees());

      rotation = m_rotationController.calculate(m_driveTrain.getYaw(), m_targetPoseFieldRelative.getRotation().getDegrees());
      SmartDashboard.putNumber("Alignment/Data/HeadingError", m_rotationController.getPositionError());
      SmartDashboard.putBoolean("Alignment/Data/atRotationSetpoint", m_rotationController.atSetpoint());


      if (!m_driverOverrideY) {
        
        m_driveYController.reset(currentPose.getY(), m_driveTrain.getState().Speeds.vxMetersPerSecond);
        driveY = m_driveYController.calculate(currentPose.getY(), m_targetPoseFieldRelative.getY());
        driveY = MathUtil.clamp(driveY, -1, 1);
        SmartDashboard.putNumber("Alignment/Data/YError", m_driveYController.getPositionError());
        SmartDashboard.putBoolean("Alignment/Data/atYSetpoint", m_driveYController.atSetpoint());
      }

      if (!m_driverOverrideX) {
        m_driveXController.reset(currentPose.getX(), m_driveTrain.getState().Speeds.vyMetersPerSecond);
        driveX = m_driveXController.calculate(currentPose.getX(), m_targetPoseFieldRelative.getX());
        driveX = MathUtil.clamp(driveX, -1, 1);
        SmartDashboard.putNumber("Alignment/Data/DistanceError", m_driveXController.getPositionError());
        SmartDashboard.putBoolean("Alignment/Data/atDistanceSetpoint", m_driveXController.atSetpoint());
      }
    }

    if (m_allowDriverOverride) {
        //override with joystick input if present
      if(m_xInput != null && Math.abs(m_xInput.getAsDouble()) > TunerConstants.DEADBAND_FACTOR){
        m_driverOverrideX = true;
        driveX = Preferences.xySlowLimitPreference.getValue()*m_xInput.getAsDouble();
      } else if (m_driverOverrideX && (m_xInput == null || Math.abs(m_xInput.getAsDouble()) <= TunerConstants.DEADBAND_FACTOR)){
        driveX = 0;
      }

      if(m_yInput != null && Math.abs(m_yInput.getAsDouble()) > TunerConstants.DEADBAND_FACTOR){
        m_driverOverrideY = true;
        driveY = Preferences.xySlowLimitPreference.getValue()*m_yInput.getAsDouble();
      } else if (m_driverOverrideY && (m_xInput == null || Math.abs(m_yInput.getAsDouble()) <= TunerConstants.DEADBAND_FACTOR)){
        driveY = 0;
      }
    }

    SmartDashboard.putNumber("Alignment/Power/rotation", rotation);
    SmartDashboard.putNumber("Alignment/Power/driveX", driveX);
    SmartDashboard.putNumber("Alignment/Power/driveY", driveY);
    
    m_driveTrain.driveRobotCentric(driveX, driveY, rotation);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_driveTrain.driveRobotCentric(0, 0,0);
  }


  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (m_allowDriverOverride){
      return false;
    } else {
      return m_rotationController.atSetpoint() && m_driveXController.atSetpoint() && m_driveYController.atSetpoint();
    }
  }
}
