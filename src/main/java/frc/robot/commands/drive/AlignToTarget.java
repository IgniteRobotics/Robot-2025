// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import java.util.Optional;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Preferences;
import frc.robot.PreferenceTypes.DoublePreference;
import frc.robot.subsystems.drive.CameraConstants;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.drive.PhotonCameraWrapper;
import frc.robot.subsystems.drive.PhotonCameraWrapper.TargetInfo;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.epilogue.Logged;

@Logged
public class AlignToTarget extends Command {
  private final CommandSwerveDrivetrain m_drive;
  private final PhotonCameraWrapper m_camera;
  private final int selectedTargetID;
  private final DoublePreference adjustment;
  

  PIDController rotationController;
  PIDController driveYController;
  PIDController driveXController;
  AprilTagFieldLayout aprilTags = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

  /** Creates a new AlignToTarget. */
  public AlignToTarget(CommandSwerveDrivetrain drive, PhotonCameraWrapper camera, int targetID, DoublePreference adj){
    m_drive = drive;
    m_camera = camera;
    selectedTargetID = targetID;
    adjustment = adj;
    addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    rotationController = new PIDController(Preferences.alignRotKP.get(), 0, Preferences.alignRotKD.get());
    driveYController = new PIDController(Preferences.alignDriveYKP.get(), 0, Preferences.alignDriveYKD.get());
    driveXController = new PIDController(Preferences.alignDriveXKP.get(), 0, Preferences.alignDriveXKD.get());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Optional<TargetInfo> targeting = m_camera.seekTarget(selectedTargetID);
    double rotation;
    double driveX;
    double driveY;

    if(targeting.isPresent()){
      double targetHeading = Math.toDegrees(aprilTags.getTagPose(selectedTargetID).get().getRotation().rotateBy(new Rotation3d(0,0,Math.PI)).getZ());
      rotation = rotationController.calculate(m_drive.getYaw(), targetHeading);
      SmartDashboard.putNumber("Alignment/Data/Heading", targetHeading);

      double offset = CameraConstants.offsetToBumper.get(targeting.get().getCameraName());
      driveX = driveXController.calculate(targeting.get().getDistance(), offset);
      SmartDashboard.putNumber("Alignment/Data/Distance", targeting.get().getDistance());
      
      driveY = -driveYController.calculate(targeting.get().getYaw(), adjustment.getValue());
      SmartDashboard.putNumber("Alignment/Data/Yaw", targeting.get().getYaw());
    }

    else{
      rotation = 0;
      driveX = 0;
      driveY = 0;
    }

    SmartDashboard.putNumber("Alignment/Power/rotation", rotation);
    SmartDashboard.putNumber("Alignment/Power/driveX", driveX);
    SmartDashboard.putNumber("Alignment/Power/driveY", driveY);
    
    m_drive.driveRobotCentric(driveX, driveY, rotation);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
