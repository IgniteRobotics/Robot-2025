// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Preferences;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import edu.wpi.first.epilogue.Logged;


@Logged
public class ManualDriveAlignment extends Command {
  private final CommandSwerveDrivetrain m_drive;
  
  
  private final DoubleSupplier m_xInput;
  private final DoubleSupplier m_yInput;
  private final DoubleSupplier m_rInput;

  
  private double m_rotation = 0;
  private double m_driveX = 0;
  private double m_driveY = 0;
  
  /** Creates a new AlignToTarget. */
  public ManualDriveAlignment(CommandSwerveDrivetrain drive, DoubleSupplier xInput, DoubleSupplier yInput, DoubleSupplier rInput){
    m_drive = drive;
    m_xInput = xInput;
    m_yInput = yInput;
    m_rInput = rInput;
    addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //Optional<TargetInfo> targeting = m_pcw.seekGeneralTargets(targetIDs, m_cameraId);
    
    //override with joystick input if present
    if(m_xInput != null && Math.abs(m_xInput.getAsDouble()) > TunerConstants.DEADBAND_FACTOR){
      m_driveX = Preferences.xySlowLimitPreference.getValue()*m_xInput.getAsDouble();
    } else {
      m_driveX = 0;
    }

    if(m_yInput != null && Math.abs(m_yInput.getAsDouble()) > TunerConstants.DEADBAND_FACTOR){
      m_driveY = Preferences.xySlowLimitPreference.getValue()*m_yInput.getAsDouble();
    } else {
      m_driveY = 0;
    }

    if (m_rInput != null && Math.abs(m_rInput.getAsDouble()) > TunerConstants.DEADBAND_FACTOR){
      m_rotation = Preferences.rotationSlowLimitPreference.getValue()*m_rInput.getAsDouble();
    } else {
      m_rotation = 0;
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
