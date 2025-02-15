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
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Preferences.DoublePreference;
import frc.robot.commands.drive.AlignToTarget;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.EndEffector.EndEffector;
@Logged
public class RobotContainer {

    private final PreferenceContainer m_preferences = new PreferenceContainer();

    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final Telemetry logger = new Telemetry(TunerConstants.DrivetrainConstants.kSpeedAt12Volts.in(MetersPerSecond));

    private final CommandXboxController joystick = new CommandXboxController(0);

    public final double default_Max_Speed = TunerConstants.DrivetrainConstants.kSpeedAt12Volts.in(MetersPerSecond);
    public final double maxAngularRate = TunerConstants.DrivetrainConstants.MAX_ANGULAR_SPEED;
    public final double deadband = TunerConstants.DrivetrainConstants.DEADBAND_FACTOR;

    private DoublePreference alignDistanceAdjustment = new DoublePreference("alignCommand/distanceAdjustment");
    
    SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
        .withDeadband(default_Max_Speed*deadband).withRotationalDeadband(maxAngularRate * deadband) // Add a 10% deadband
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);;

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.DrivetrainConstants.createDrivetrain();

    public final Elevator elevator = new Elevator();

    public final EndEffector endEffector = new EndEffector();

    private final Command alignTest = new AlignToTarget(drivetrain,drivetrain.m_photonCameraWrapper, 12, alignDistanceAdjustment);


    /* Path follower */
    private final SendableChooser<Command> autoChooser;

    public DoublePreference armLength = new DoublePreference("armLength", 1);
    public DoublePreference wristAngle = new DoublePreference("wristAngle", 90);

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

        joystick.x().onTrue(AutoBuilder.followPath(createTestPath()));

        /* 
        joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        joystick.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        ));
        */
        joystick.a().whileTrue(new RunCommand(() -> elevator.setPositionRevolutions(m_preferences.elevatorPosition)));
        joystick.b().onTrue(new InstantCommand(() -> elevator.setElevatorPID(m_preferences.elevatorkP, m_preferences.elevatorkD, m_preferences.elevatorkI, m_preferences.elevatorkG)));


        joystick.pov(0).whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(0.5).withVelocityY(0))
        );
        joystick.pov(180).whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(-0.5).withVelocityY(0))
        );

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        //joystick.rightBumper().onTrue(new InstantCommand( () -> elevator.alterMech(armLength.getValue(), wristAngle.getValue())));

        drivetrain.registerTelemetry(logger::telemeterize);
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
