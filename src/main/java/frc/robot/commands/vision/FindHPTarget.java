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
public class FindHPTarget extends Command {
  private final PhotonCameraWrapper m_photonCameraWrapper;
  private final Supplier<int[]> m_idSupplier;
  private final Supplier<PhotonCamera> m_cameraSupplier;
  private int lockedTarget = -1;
  AprilTagFieldLayout aprilTags = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

  /** Creates a new FindReefTarget. */
  public FindHPTarget(PhotonCameraWrapper photonCameraWrapper, Supplier<int[]> idSupplier, Supplier<PhotonCamera> cameraSupplier) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_photonCameraWrapper = photonCameraWrapper;
    m_idSupplier = idSupplier;
    m_cameraSupplier = cameraSupplier;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    DriveState.getInstance().setTargetPose2d(null);
    lockedTarget = -1;
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

        //camera to target
        Transform2d cam2Target = new Transform2d(
          targeting.get().getTransform3d().getTranslation().getX(),
          targeting.get().getTransform3d().getTranslation().getY(),
          targeting.get().getTransform3d().getRotation().toRotation2d()
        );

        //robot to camera.
        Transform2d robotToCamera = new Transform2d(
          CoralState.getInstance().getCameraTransform().getX(),
          CoralState.getInstance().getCameraTransform().getY(),
          CoralState.getInstance().getCameraTransform().getRotation().toRotation2d()
        );

        //robot offset the center of the robot to position for intake
        // don't rotate since the robot to cam transofrm is already rotated.
        Transform2d robotToIntakePosition = new Transform2d(
          (Math.abs(CameraConstants.X_OFFSET_METERS) + Math.abs(CoralState.getInstance().getCameraTransform().getX())),
          0,
          new edu.wpi.first.math.geometry.Rotation2d()
        );

        //start at the robot, moveto the camera, then to the target's position, then modify by the offsets
        Pose2d targetPose = DriveState.getInstance().getPose2d()
          .plus(robotToCamera).plus(cam2Target).plus(robotToIntakePosition);

        DriveState.getInstance().setTargetPose2d(targetPose);
        //finished = true;
      }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // DriveState.getInstance().setTargetPose2d(null);
    DriveState.getInstance().unlockCameras();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

