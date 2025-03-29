// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.test;

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
public class RotateToHeading extends Command {
  private final CommandSwerveDrivetrain m_drive;
  PhotonCameraWrapper m_pcw;
  PIDController rotationController;
  PIDController driveYController;
  PIDController driveXController;
  AprilTagFieldLayout aprilTags = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
  
  private int targetIDs[] = {};

  double rotation;
  
  /** Creates a new AlignToTarget. */
  public RotateToHeading(CommandSwerveDrivetrain drive,  PhotonCameraWrapper pcw){
    m_drive = drive;
    m_pcw = pcw;
    addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    rotationController = new PIDController(Preferences.alignRotKP.get(), Preferences.alignRotKI.get(), Preferences.alignRotKD.get());
    rotationController.setTolerance(Preferences.rotationTolerancePreference.get());
    rotationController.enableContinuousInput(-180, 180);

    targetIDs = AllianceState.getInstance().getReefTags();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    rotation = 0;

    Optional<TargetInfo> targeting = m_pcw.seekTargets(targetIDs, CoralState.getInstance().pickReefCamera());

    if(targeting.isPresent()){
      double targetHeading = Math.toDegrees(aprilTags.getTagPose(targeting.get().getTagId()).get().getRotation().getZ());
      rotation = rotationController.calculate(m_drive.getYaw(), targetHeading);
      SmartDashboard.putNumber("Alignment/Data/Heading", targetHeading);
    }

    SmartDashboard.putNumber("Alignment/Power/rotation", rotation);
    
    m_drive.driveRobotCentric(0, 0, rotation);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.driveRobotCentric(0,0,0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return rotationController.atSetpoint();
  }
}
