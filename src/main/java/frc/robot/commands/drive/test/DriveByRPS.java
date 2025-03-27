// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive.test;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.PreferenceTypes.DoublePreference;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveByRPS extends Command {
  private DoublePreference m_driveRPS;
  private CommandSwerveDrivetrain m_drivetrain;

  private final SwerveRequest.RobotCentric m_driveRequest = new SwerveRequest.RobotCentric()
   .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
   .withSteerRequestType(SteerRequestType.MotionMagicExpo);

public DriveByRPS(CommandSwerveDrivetrain drivetrain, DoublePreference rps) {  
    m_drivetrain = drivetrain;
    m_driveRPS = rps;
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  @Override
  public void execute() {
    
    m_drivetrain.getModule(0).getDriveMotor().setControl(new VelocityVoltage(m_driveRPS.getValue()));
    m_drivetrain.getModule(1).getDriveMotor().setControl(new VelocityVoltage(m_driveRPS.getValue()));
    m_drivetrain.getModule(2).getDriveMotor().setControl(new VelocityVoltage(m_driveRPS.getValue()));
    m_drivetrain.getModule(3).getDriveMotor().setControl(new VelocityVoltage(m_driveRPS.getValue()));

  }
  

  // Called once the command ends or is .interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.setControl(m_driveRequest.withVelocityX(0).withVelocityX(0).withRotationalRate(0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
