// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import java.util.Optional;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Preferences.DoublePreference;
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
  private final DoublePreference xError;
  
  private DoublePreference alignRotKP = new DoublePreference("alignCommand/rotation/kP", 0);
  private DoublePreference alignRotKD = new DoublePreference("alignCommand/rotation/kD", 0);
  private DoublePreference alignDriveXKP = new DoublePreference("alignCommand/drive/x/kP", 0);
  private DoublePreference alignDriveXKD = new DoublePreference("alignCommand/drive/x/kD", 0);
  private DoublePreference alignDriveYKP = new DoublePreference("alignCommand/drive/y/kP", 0);
  private DoublePreference alignDriveYKD = new DoublePreference("alignCommand/drive/y/KD", 0);

  PIDController rotationController;
  PIDController driveXController;
  PIDController driveYController;

  AprilTagFieldLayout aprilTags = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

  /** Creates a new AlignToTarget. */
  public AlignToTarget(CommandSwerveDrivetrain drive, PhotonCameraWrapper camera, int targetID, DoublePreference error){
    m_drive = drive;
    m_camera = camera;
    selectedTargetID = targetID;
    xError = error;
    addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    rotationController = new PIDController(alignRotKP.get(), 0, alignRotKD.get());
    driveXController = new PIDController(alignDriveXKP.get(), 0, alignDriveXKD.get());
    driveYController = new PIDController(alignDriveYKP.get(), 0, alignDriveYKD.get());
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
      rotation = rotationController.calculate(m_drive.getYaw() - targetHeading, 0);

      double offset = CameraConstants.offsetToBumper.get(targeting.get().getCameraName());
      driveX = driveXController.calculate(targeting.get().getDistance() - offset - xError.get(), 0);
      
      driveY = driveYController.calculate(targeting.get().getYaw(), 0);
    }

    else{
      rotation = 0;
      driveX = 0;
      driveY = 0;
    }
    
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
