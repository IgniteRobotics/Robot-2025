// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.composite;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Preferences;
import frc.robot.RobotState;
import frc.robot.commands.elevator.ToSetpoint;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.Elevator.ElevatorConstants;
import frc.robot.subsystems.EndEffector.EndEffector;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Score extends SequentialCommandGroup {
  /** Creates a new CommandDispatcher. */
  public Score(RobotState robotState, 
                           Elevator elevator, 
                           EndEffector endEffector, 
                           CommandSwerveDrivetrain drive) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    if (robotState.hasCoral() && robotState.getCoralHeight() != 0){
      addCommands(new ScoreCoral(elevator, endEffector, robotState.getCoralHeight(), ElevatorConstants.FLOOR.GROUND.position));
    } else if (robotState.hasAlgae() && robotState.getAlgaeHeight() != 0){
      addCommands(
        new ParallelCommandGroup(
          new ToSetpoint(elevator, robotState.getAlgaeHeight()),
          //TODO:  Add wrist positions
          new RunCommand(() -> endEffector.setWristPosition(robotState.getAlgaeWristPosition()))
            .withName("SetWristPosition")
            .until(() -> endEffector.isWristAtPosition())
        ),
        new RunCommand(()-> endEffector.outtakeAlgae())
          .withName("EjectAlgae")
          .withTimeout(.50),
        new ParallelCommandGroup(
          new ToSetpoint(elevator, ElevatorConstants.FLOOR.GROUND.position),
          new RunCommand(() -> endEffector.setWristPosition(robotState.getAlgaeWristPosition()))
            .withName("SetWristPosition")
            .until(() -> endEffector.isWristAtPosition())
        )
      );

    } else {

    }
  }
}
