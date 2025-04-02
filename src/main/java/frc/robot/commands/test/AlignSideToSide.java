// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.test;

import java.util.Optional;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
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
public class AlignSideToSide extends Command {
  private final CommandSwerveDrivetrain m_drive;
  PhotonCameraWrapper m_pcw;
  ProfiledPIDController driveYController;
  AprilTagFieldLayout aprilTags = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
  Constraints m_YConstraints;
  
  private int targetIDs[] = {};
  
  /** Creates a new AlignToTarget. */
  public AlignSideToSide(CommandSwerveDrivetrain drive,  PhotonCameraWrapper pcw){
    m_drive = drive;
    m_pcw = pcw;
    addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_YConstraints = new Constraints(Preferences.profiledDriveYMaxVel.get(), Preferences.profiledDriveYMaxAcc.get());
    driveYController = new ProfiledPIDController(Preferences.profiledDriveYKP.get(), Preferences.profiledDriveYKI.get(), Preferences.profiledDriveYKD.get(), m_YConstraints);
    driveYController.setTolerance(Preferences.yAlignTolerancePreference.get());
    driveYController.setIZone(Double.POSITIVE_INFINITY);
  
    targetIDs = AllianceState.getInstance().getReefTags();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //Optional<TargetInfo> targeting = m_pcw.seekGeneralTargets(targetIDs, m_cameraId);
    
    Optional<TargetInfo> targeting = m_pcw.seekTargets(targetIDs, CoralState.getInstance().pickReefCamera());
    double driveY;

    if(targeting.isPresent()){
      driveY = driveYController.calculate(targeting.get().getTransform3d().getY(), 0);
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
