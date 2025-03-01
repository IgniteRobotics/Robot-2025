// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import java.util.Optional;
import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Preferences;
import frc.robot.PreferenceTypes.DoublePreference;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.drive.PhotonCameraWrapper;
import frc.robot.subsystems.drive.PhotonCameraWrapper.TargetInfo;

public class TurnToTarget extends Command {
  private final CommandSwerveDrivetrain m_drive;
  private final PhotonCameraWrapper m_camera;
  private final Supplier<Double> m_driveX;
  private final int m_target;
  PIDController rotationController;

  
  public TurnToTarget(CommandSwerveDrivetrain drive, PhotonCameraWrapper camera, Supplier<Double> supplier, int target) {
    m_drive = drive;
    m_camera = camera;
    m_driveX = supplier;
    m_target = target;
    addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    rotationController = new PIDController(Preferences.turnToTarget_rotKP.getValue(), 0, Preferences.turnToTarget_rotKD.getValue());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Optional<TargetInfo> targeting = m_camera.seekTarget(m_target);
    double rotation = 0.0;
    if(targeting.isPresent()){
      rotation = rotationController.calculate(targeting.get().getYaw(), 0);
    }

    m_drive.driveRobotCentric(m_driveX.get(), 0, rotation);
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
