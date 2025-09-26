// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Preferences;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain.Wheel;
import edu.wpi.first.epilogue.Logged;


@Logged
public class SlipTestIndividual extends Command {
  private final CommandSwerveDrivetrain m_drive;
  private final Wheel m_wheel;
  private double step;
  private ScheduledExecutorService scheduler = Executors.newSingleThreadScheduledExecutor();

  /** Creates a new AlignToTarget. */
  public SlipTestIndividual(CommandSwerveDrivetrain drive, Wheel wheel){
    m_drive = drive;
    m_wheel = wheel;
    addRequirements(m_drive);
  }

  public void incrementStep(){
    step += 0.05;
    m_drive.getModule(m_wheel.getModuleInt()).getDriveMotor().set(step);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    step = 0;
    scheduler.scheduleAtFixedRate(() -> {incrementStep();}, 3, 1, TimeUnit.SECONDS);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("Test has ended");
    System.out.println( m_wheel.getName() + " current: " + m_drive.getModule(m_wheel.getModuleInt()).getDriveMotor().getStatorCurrent().getValueAsDouble());
    m_drive.setDrivePower(0);
  }


  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
     }
}
