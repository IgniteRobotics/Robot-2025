// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.List;
import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.PreferenceTypes.DoublePreference;
import frc.robot.commands.drive.AlignToTarget;
import frc.robot.commands.endeffector.AutoIngestCoral;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.Elevator.ElevatorConstants;
import frc.robot.subsystems.EndEffector.EndEffector;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.ClimberConstants;
@Logged
public class RobotContainer {

    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final Telemetry logger = new Telemetry(TunerConstants.kSpeedAt12Volts.in(MetersPerSecond));

    private final CommandXboxController joystick = new CommandXboxController(0);

    // Set up manipulator joystick
    private final Joystick manipulatorJoystick = new Joystick(1);
    private final JoystickButton coralTroughButton = new JoystickButton(manipulatorJoystick, 0);
    private final JoystickButton coralL2LeftButton = new JoystickButton(manipulatorJoystick, 1);
    private final JoystickButton coralL2RightButton = new JoystickButton(manipulatorJoystick, 2);
    private final JoystickButton coralL3LeftButton = new JoystickButton(manipulatorJoystick, 3);
    private final JoystickButton coralL3RightButton = new JoystickButton(manipulatorJoystick, 4);
    private final JoystickButton coralL4LeftButton = new JoystickButton(manipulatorJoystick, 5);
    private final JoystickButton coralL4RightButton = new JoystickButton(manipulatorJoystick, 6);
    private final JoystickButton algaeProcessorButton = new JoystickButton(manipulatorJoystick, 7);
    private final JoystickButton algaeReefButton = new JoystickButton(manipulatorJoystick, 8);
    private final JoystickButton algaeBargeButton = new JoystickButton(manipulatorJoystick, 9);
    private final JoystickButton algaeCancelButton = new JoystickButton(manipulatorJoystick, 10);
    private final JoystickButton coralCancelButton = new JoystickButton(manipulatorJoystick, 11);


    public final double default_Max_Speed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    public final double maxAngularRate = TunerConstants.MAX_ANGULAR_SPEED;
    public final double deadband = TunerConstants.DEADBAND_FACTOR;
    
    SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
        .withDeadband(default_Max_Speed*deadband).withRotationalDeadband(maxAngularRate * deadband) // Add a 10% deadband
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);;

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    public final Elevator elevator = new Elevator();

    public final EndEffector endEffector = new EndEffector();

    public final Climber climber = new Climber();

    private final Command alignTest = new AlignToTarget(drivetrain,drivetrain.m_photonCameraWrapper, 12, Preferences.alignDistanceAdjustment);

    private final Command autoIngestCoral = new AutoIngestCoral(endEffector);



    /* Path follower */
    private final SendableChooser<Command> autoChooser;

    private RobotState m_RobotState = RobotState.getInstance();
    //drive command
    //Command arcadeDrive =  new RunCommand(() -> drivetrain.drive(-joystick.getLeftY(), -joystick.getLeftX(), -joystick.getRightX(), m_RobotState.getMaxSpeed())) {{
    //   addRequirements(drivetrain);
    //}};

    public RobotContainer() {

        autoChooser = AutoBuilder.buildAutoChooser("Tests");
        SmartDashboard.putData("Auto Mode", autoChooser);

        configureBindings();
    }

    private void configureManipulatorController(){
        coralTroughButton.whileTrue(new InstantCommand(() -> m_RobotState.setCoralTarget(RobotState.CoralTarget.TROUGH)));
        coralL2LeftButton.whileTrue(new InstantCommand(() -> m_RobotState.setCoralTarget(RobotState.CoralTarget.L2_LEFT)));
        coralL2RightButton.whileTrue(new InstantCommand(() -> m_RobotState.setCoralTarget(RobotState.CoralTarget.L2_RIGHT)));
        coralL3LeftButton.whileTrue(new InstantCommand(() -> m_RobotState.setCoralTarget(RobotState.CoralTarget.L3_LEFT)));
        coralL3RightButton.whileTrue(new InstantCommand(() -> m_RobotState.setCoralTarget(RobotState.CoralTarget.L3_RIGHT)));
        coralL4LeftButton.whileTrue(new InstantCommand(() -> m_RobotState.setCoralTarget(RobotState.CoralTarget.L4_LEFT)));
        coralL4RightButton.whileTrue(new InstantCommand(() -> m_RobotState.setCoralTarget(RobotState.CoralTarget.L4_RIGHT)));
        algaeProcessorButton.whileTrue(new InstantCommand(() -> m_RobotState.setAlgaeTarget(RobotState.AlgaeTarget.PROCESSOR)));
        algaeReefButton.whileTrue(new InstantCommand(() -> m_RobotState.setAlgaeTarget(RobotState.AlgaeTarget.REEF)));
        algaeBargeButton.whileTrue(new InstantCommand(() -> m_RobotState.setAlgaeTarget(RobotState.AlgaeTarget.BARGE)));
        coralCancelButton.whileTrue(new InstantCommand(() -> m_RobotState.setCoralTarget(RobotState.CoralTarget.NONE)));
        algaeCancelButton.whileTrue(new InstantCommand(() -> m_RobotState.setAlgaeTarget(RobotState.AlgaeTarget.NONE)));
       
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * m_RobotState.getMaxSpeed()) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * m_RobotState.getMaxSpeed()) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * m_RobotState.getMaxRotation()) // Drive counterclockwise with negative X (left)
            )
        );

        endEffector.setDefaultCommand(autoIngestCoral);

        //joystick.x().onTrue(AutoBuilder.followPath(createTestPath()));
         
        // joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        // joystick.b().whileTrue(drivetrain.applyRequest(() ->
        //     point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        // ));

        //joystick.x().whileTrue(new RunCommand(() -> elevator.setPositionRevolutions(Preferences.elevatorPosition)));
        //joystick.y().onTrue(new InstantCommand(() -> elevator.setElevatorPID(Preferences.elevatorkP, Preferences.elevatorkD, Preferences.elevatorkI, Preferences.elevatorkG)));

        SmartDashboard.putData("Elevator to preset", new RunCommand(() -> elevator.setPositionRevolutions(Preferences.elevatorPosition)));
        SmartDashboard.putData("Set Elevator PID", new InstantCommand(() -> elevator.setElevatorPID(Preferences.elevatorkP, Preferences.elevatorkD, Preferences.elevatorkI, Preferences.elevatorkG, Preferences.elevatorkS)));
        SmartDashboard.putData("Set Elevator Motion Magic Configs", new InstantCommand(() -> elevator.setElevatorMotionMagic(Preferences.elevatorMMCruiseVelocity, Preferences.elevatorMMAccel, Preferences.elevatorMMJerk, Preferences.elevatorMMkV, Preferences.elevatorMMkA)));
        
        SmartDashboard.putData("Elevator Ground", new InstantCommand(() -> elevator.setPositionRevolutions(ElevatorConstants.FLOOR.GROUND.position)));
        SmartDashboard.putData("Elevator Trough", new RunCommand(() -> elevator.setPositionRevolutions(ElevatorConstants.FLOOR.TROUGH.position))
            .until(() -> elevator.atSetpoint())
            .andThen( new RunCommand(() -> endEffector.outtakeCoral())
            .withTimeout(1)
            .andThen(new InstantCommand(() -> endEffector.stopCoralMotor())))
            );
        SmartDashboard.putData("Elevator Level 2", new RunCommand(() -> elevator.setPositionRevolutions(ElevatorConstants.FLOOR.LEVEL_2.position))
            .until(() -> elevator.atSetpoint())
            .andThen( new RunCommand(() -> endEffector.outtakeCoral())
            .withTimeout(1)
            .andThen(new InstantCommand(() -> endEffector.stopCoralMotor())))
            .andThen(new InstantCommand(() -> elevator.setPositionRevolutions(ElevatorConstants.FLOOR.GROUND.position)))
            );
        SmartDashboard.putData("Elevator Level 3", new InstantCommand(() -> elevator.setPositionRevolutions(ElevatorConstants.FLOOR.LEVEL_3.position))
            .until(() -> elevator.atSetpoint())
            .andThen( new RunCommand(() -> endEffector.outtakeCoral())
            .withTimeout(1)
            .andThen(new InstantCommand(() -> endEffector.stopCoralMotor())))
            .andThen(new InstantCommand(() -> elevator.setPositionRevolutions(ElevatorConstants.FLOOR.GROUND.position)))
            );

        SmartDashboard.putData("Elevator Level 4", new InstantCommand(() -> elevator.setPositionRevolutions(ElevatorConstants.FLOOR.LEVEL_4.position))
            .until(() -> elevator.atSetpoint())
            .andThen( new RunCommand(() -> endEffector.outtakeCoral())
            .withTimeout(1)
            .andThen(new InstantCommand(() -> endEffector.stopCoralMotor())))
            .andThen(new InstantCommand(() -> elevator.setPositionRevolutions(ElevatorConstants.FLOOR.GROUND.position)))
            );

        SmartDashboard.putData("Outtake", new RunCommand(() -> endEffector.outtakeCoral()).withTimeout(1).andThen(new InstantCommand(() -> endEffector.stopCoralMotor())));
        joystick.pov(0).whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(0.5).withVelocityY(0))
        );
        joystick.pov(180).whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(-0.5).withVelocityY(0))
        );

        SmartDashboard.putData("Climber to preset", new InstantCommand(() -> climber.setPositionRevolutions(Preferences.climberPosition)));
        SmartDashboard.putData("Set Climber PID", new InstantCommand(() -> climber.setClimberPID(Preferences.climberkP, Preferences.climberkD, Preferences.climberkI, Preferences.climberkG)));
        SmartDashboard.putData("Set Climber Motion Magic Configs", new InstantCommand(() -> climber.setClimberMotionMagic(Preferences.climberMMCruiseVelocity, Preferences.climberMMAccel, Preferences.climberMMJerk, Preferences.climberMMkV, Preferences.climberMMkA)));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        joystick.a().onTrue(new InstantCommand(() -> climber.setSpeed(Preferences.climberUpSpeed)))
                    .onFalse(new InstantCommand(() -> climber.setSpeed(0)));
        joystick.b().onTrue(new InstantCommand(() -> climber.setSpeed(Preferences.climberDownSpeed)))
                    .onFalse(new InstantCommand(() -> climber.setSpeed(0)));
        joystick.x().whileTrue(new InstantCommand(() -> climber.setSpeed(0)));

        // reset the field-centric heading on left bumper press
        joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        //joystick.rightBumper().onTrue(new InstantCommand( () -> elevator.alterMech(armLength.getValue(), wristAngle.getValue())));

        drivetrain.registerTelemetry(logger::telemeterize);

        configureManipulatorController();
    }

    public Command getAutonomousCommand() {
        /* First put the drivetrain into auto run mode, then run the auto */
        return autoChooser.getSelected();
    }

    public PathPlannerPath createTestPath(){
        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
        new Pose2d(1.0, 1.0, Rotation2d.fromDegrees(0)),
        new Pose2d(3.0, 1.0, Rotation2d.fromDegrees(0)),
        new Pose2d(5.0, 3.0, Rotation2d.fromDegrees(90))
            );

        PathConstraints constraints = new PathConstraints(3.0, 3.0, 2 * Math.PI, 4 * Math.PI); // The constraints for this path.
        // PathConstraints constraints = PathConstraints.unlimitedConstraints(12.0); // You can also use unlimited constraints, only limited by motor torque and nominal battery voltage

        // Create the path using the waypoints created above
        PathPlannerPath path = new PathPlannerPath(
              waypoints,
               constraints,
               null, // The ideal starting state, this is only relevant for pre-planned paths, so can be null for on-the-fly paths.
                new GoalEndState(0, Rotation2d.fromDegrees(-90)) // Goal end state. You can set a holonomic rotation here. If using a differential drivetrain, the rotation will have no effect.
        );

        // Prevent the path from being flipped if the coordinates are already correct
        path.preventFlipping = true;

        return path;
    }
}
