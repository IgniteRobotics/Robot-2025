// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.elevator;

import java.security.Policy;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator.Elevator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TestElevatorPower extends Command {
  private Elevator m_Elevator;
  private DoubleSupplier m_powSupplier;
  /** Creates a new TestElevatorPower. */
  public TestElevatorPower(Elevator elevator, DoubleSupplier powSupplier) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_Elevator = elevator;
    m_powSupplier = powSupplier;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_Elevator.stopMotors();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_Elevator.setPower(m_powSupplier.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Elevator.stopMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
