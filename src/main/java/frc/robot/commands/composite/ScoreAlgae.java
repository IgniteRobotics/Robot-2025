// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.composite;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.elevator.ToSetpoint;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.algae.AlgaeCollector;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoreAlgae extends ParallelCommandGroup {
  /** Creates a new ScoreAlgae. */
  public ScoreAlgae(Elevator elevator, AlgaeCollector collector,
                    double elevatorHeight,  double wristAngle, Trigger eject) {
    addCommands(
      new ToSetpoint(elevator, elevatorHeight),
      new RunCommand(() -> collector.setWristPosition(wristAngle)),
      new WaitUntilCommand(eject).andThen(new RunCommand(() -> collector.outtakeAlgae()))
    );
  }
}
