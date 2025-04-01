// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.vision;

import java.util.Optional;
import java.util.function.Supplier;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.statemachines.CoralState;
import frc.robot.statemachines.DriveState;
import frc.robot.subsystems.drive.CameraConstants;
import frc.robot.subsystems.drive.PhotonCameraWrapper;
import frc.robot.subsystems.drive.PhotonCameraWrapper.TargetInfo;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class FindReefTarget extends Command {
  private final PhotonCameraWrapper m_photonCameraWrapper;
  private final Supplier<int[]> m_idSupplier;
  private final Supplier<PhotonCamera> m_cameraSupplier;
  private int lockedTarget = -1;
  AprilTagFieldLayout aprilTags = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
  private boolean finished = false;

  /** Creates a new FindReefTarget. */
  public FindReefTarget(PhotonCameraWrapper photonCameraWrapper, Supplier<int[]> idSupplier, Supplier<PhotonCamera> cameraSupplier) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_photonCameraWrapper = photonCameraWrapper;
    m_idSupplier = idSupplier;
    m_cameraSupplier = cameraSupplier;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    DriveState.getInstance().setTargetPose2d(null);
    finished = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Optional<TargetInfo> targeting;
    if(lockedTarget == -1)
      targeting = m_photonCameraWrapper.seekTargets(m_idSupplier.get(), m_cameraSupplier.get());
    else 
      targeting = m_photonCameraWrapper.seekTargets(lockedTarget, m_cameraSupplier.get()); 

      if(targeting.isPresent()){

        lockedTarget = targeting.get().getTagId();
        DriveState.getInstance().lockToCamera(targeting.get().getCamera());

        //modify the target's transform by the offsets

        
        Transform2d targetTransform = new Transform2d(
          targeting.get().getTransform3d().getTranslation().getX() - CameraConstants.X_OFFSET_METERS,
          targeting.get().getTransform3d().getTranslation().getY() + CoralState.getInstance().getYCoralOffsetMeters(),
          targeting.get().getTransform3d().getRotation().toRotation2d()
        );

        Transform2d cam2Robot = new Transform2d(
          CoralState.getInstance().getCameraTransform().getX(),
          CoralState.getInstance().getCameraTransform().getY(),
          CoralState.getInstance().getCameraTransform().getRotation().toRotation2d()
        );

        //start at the robot, moved to the camera, then to the target's position, then modify by the offset
        Pose2d targetPose = DriveState.getInstance().getPose2d()
          .plus(cam2Robot).plus(targetTransform);

        DriveState.getInstance().setTargetPose2d(targetPose);
        finished = true;

      }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // DriveState.getInstance().setTargetPose2d(null);
    // DriveState.getInstance().unlockCameras();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }
}
