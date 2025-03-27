// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive.test;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TurnByAngle extends Command {

  private double m_currentAngle;
  private DoubleSupplier m_increment;
  private CommandSwerveDrivetrain m_drivetrain;
  private final SwerveRequest.PointWheelsAt point;

  /** Creates a new TurnByAngle. */
  public TurnByAngle(CommandSwerveDrivetrain drivetrain, double initialAngle, DoubleSupplier increment) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_increment = increment;
    m_currentAngle = initialAngle;
    m_drivetrain = drivetrain;
    addRequirements(drivetrain);
    point = new SwerveRequest.PointWheelsAt();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drivetrain.applyRequest(() -> point.withModuleDirection(new Rotation2d(m_currentAngle)));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("test/drive/steer/targetAngle", m_currentAngle);
    m_drivetrain.applyRequest(() -> point.withModuleDirection(new Rotation2d(m_currentAngle)));
    m_currentAngle += m_increment.getAsDouble() % 360;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.applyRequest(() -> point.withModuleDirection(new Rotation2d(0)));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
