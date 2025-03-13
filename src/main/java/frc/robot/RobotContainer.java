// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
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
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.PreferenceTypes.DoublePreference;
import frc.robot.commands.auton.AutonComposites;
import frc.robot.commands.composite.AutoScoreCoralGroup;
import frc.robot.commands.composite.IntakeAlgae;
import frc.robot.commands.composite.OuttakeAlgae;
import frc.robot.commands.composite.Score;
import frc.robot.commands.composite.ScoreAlgae;
import frc.robot.commands.composite.ScoreCoral;
import frc.robot.commands.composite.SemiAutoScoreCoralGroup;
import frc.robot.commands.corraler.CorralerDefaultCommand;
import frc.robot.commands.corraler.OuttakeCommand;
import frc.robot.commands.drive.AlignIntakeSide;
import frc.robot.commands.drive.AlignSideToSide;
import frc.robot.commands.drive.AlignThenDrive;
import frc.robot.commands.drive.AlignToReefTags;
import frc.robot.commands.drive.DriveIntoTarget;
import frc.robot.commands.drive.RotateToHeading;
import frc.robot.commands.elevator.ElevatorToAlgaePreset;
import frc.robot.commands.elevator.ToSetpoint;
import frc.robot.generated.TunerConstants;
import frc.robot.statemachines.AllianceState;
import frc.robot.statemachines.CoralState;
import frc.robot.statemachines.DriveState;
import frc.robot.statemachines.AlgaeState;
import frc.robot.statemachines.CoralState.CoralTarget;
import frc.robot.subsystems.drive.CameraConstants;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.drive.PhotonCameraWrapper;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.Elevator.ElevatorConstants;
import frc.robot.subsystems.algae.AlgaeCollector;
import frc.robot.subsystems.algae.AlgaeCollectorConstants;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.ClimberConstants;
import frc.robot.subsystems.coral.Corraler;
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
    private final JoystickButton coralTroughButton = new JoystickButton(manipulatorJoystick, 1);
    private final JoystickButton coralL2LeftButton = new JoystickButton(manipulatorJoystick, 2);
    private final JoystickButton coralL2RightButton = new JoystickButton(manipulatorJoystick, 3);
    private final JoystickButton coralL3LeftButton = new JoystickButton(manipulatorJoystick, 4);
    private final JoystickButton coralL3RightButton = new JoystickButton(manipulatorJoystick, 5);
    private final JoystickButton coralL4LeftButton = new JoystickButton(manipulatorJoystick, 6);
    private final JoystickButton coralL4RightButton = new JoystickButton(manipulatorJoystick, 7);
    private final JoystickButton algaeProcessorButton = new JoystickButton(manipulatorJoystick, 8);
    private final JoystickButton algaeReefLowButton = new JoystickButton(manipulatorJoystick, 9);
    private final JoystickButton algaeReefHighButton = new JoystickButton(manipulatorJoystick, 10);
    private final JoystickButton algaeBargeButton = new JoystickButton(manipulatorJoystick, 11);
    private final JoystickButton coralCancelButton = new JoystickButton(manipulatorJoystick, 12);


    public final double default_Max_Speed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    public final double maxAngularRate = TunerConstants.MAX_ANGULAR_SPEED;
    public final double deadband = TunerConstants.DEADBAND_FACTOR;
    
    SwerveRequest.FieldCentric 
    drive = new SwerveRequest.FieldCentric()
        .withDeadband(default_Max_Speed*deadband).withRotationalDeadband(maxAngularRate * deadband) // Add a 10% deadband
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);;

    PhotonCameraWrapper m_PhotonCameraWrapper = new PhotonCameraWrapper();

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain(m_PhotonCameraWrapper);

    public final Elevator elevator = new Elevator();

    public final Corraler corraler = new Corraler();

    public final AlgaeCollector collector = new AlgaeCollector();

    public final Climber climber = new Climber();

    private final Command corralerDefaultCommand = new CorralerDefaultCommand(corraler);


    /* Path follower */
    private final SendableChooser<Command> autoChooser;

    private DriveState m_driveState = DriveState.getInstance();

    private AllianceState m_allianceState = AllianceState.getInstance();

    private CoralState m_CoralState = CoralState.getInstance();

    private AlgaeState m_AlgaeState = AlgaeState.getInstance();
    


    public RobotContainer() {
        
        NamedCommands.registerCommand("Score Level 4 at Reef K", AutonComposites.ScoreLevel4ReefK(drivetrain, m_PhotonCameraWrapper, elevator, corraler));
        NamedCommands.registerCommand("Score Level 4 at Reef L", AutonComposites.ScoreLevel4ReefL(drivetrain, m_PhotonCameraWrapper, elevator, corraler));
        NamedCommands.registerCommand("Score Level 4 at Reef J", AutonComposites.ScoreLevel4ReefJ(drivetrain, m_PhotonCameraWrapper, elevator, corraler));
        NamedCommands.registerCommand("Intake at HP", AutonComposites.IntakeCoralHP(drivetrain, m_PhotonCameraWrapper, elevator));
        

        autoChooser = AutoBuilder.buildAutoChooser("Auto Chooser");
        autoChooser.addOption("3 Coral Auton", AutoBuilder.buildAuto("3 Coral Auton"));
        autoChooser.addOption("Simple Drive Auton", AutoBuilder.buildAuto("Simple Auton"));
        
        autoChooser.addOption("Line Up and Score", new RunCommand(() -> drivetrain.driveRobotCentric(-1, 0, 0)).withTimeout(2)
            .alongWith(new ScoreCoral(elevator, corraler, ElevatorConstants.FLOOR.LEVEL_4.position, ElevatorConstants.FLOOR.GROUND.position)));
        SmartDashboard.putData("Auto Mode", autoChooser);


        configureSubsytemDefaultCommands();
        // configureBindings();
        //TODO MUST REMOVE
        configureTestBindings();
        configureManipulatorController(); 
        drivetrain.registerTelemetry(logger::telemeterize);
    }

    private void configureManipulatorController(){
        coralTroughButton.onTrue(new InstantCommand(() -> m_CoralState.setCoralTarget(CoralState.CoralTarget.TROUGH)));
        coralL2LeftButton.onTrue(new InstantCommand(() -> m_CoralState.setCoralTarget(CoralState.CoralTarget.L2_LEFT)));
        coralL2RightButton.onTrue(new InstantCommand(() -> m_CoralState.setCoralTarget(CoralState.CoralTarget.L2_RIGHT)));
        coralL3LeftButton.onTrue(new InstantCommand(() -> m_CoralState.setCoralTarget(CoralState.CoralTarget.L3_LEFT)));
        coralL3RightButton.onTrue(new InstantCommand(() -> m_CoralState.setCoralTarget(CoralState.CoralTarget.L3_RIGHT)));
        coralL4LeftButton.onTrue(new InstantCommand(() -> m_CoralState.setCoralTarget(CoralState.CoralTarget.L4_LEFT)));
        coralL4RightButton.onTrue(new InstantCommand(() -> m_CoralState.setCoralTarget(CoralState.CoralTarget.L4_RIGHT)));
        algaeProcessorButton.onTrue(new InstantCommand(() -> m_AlgaeState.setAlgaeTarget(AlgaeState.AlgaeTarget.PROCESSOR)));
        algaeReefLowButton.onTrue(new InstantCommand(() -> m_AlgaeState.setAlgaeTarget(AlgaeState.AlgaeTarget.LOW_REEF)));
        algaeReefHighButton.onTrue(new InstantCommand(() -> m_AlgaeState.setAlgaeTarget(AlgaeState.AlgaeTarget.HIGH_REEF)));
        algaeBargeButton.onTrue(new InstantCommand(() -> m_AlgaeState.setAlgaeTarget(AlgaeState.AlgaeTarget.BARGE)));
        coralCancelButton.onTrue(new InstantCommand(() -> m_CoralState.setCoralTarget(CoralState.CoralTarget.NONE)));
       
    }

    private void configureSubsytemDefaultCommands(){
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
         
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * m_driveState.getMaxSpeed()) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * m_driveState.getMaxSpeed()) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * m_driveState.getMaxRotation()) // Drive counterclockwise with negative X (left)
            )
        );
        

        corraler.setDefaultCommand(corralerDefaultCommand);

        

        // elevator.setDefaultCommand(new ToSetpoint(elevator, ElevatorConstants.FLOOR.GROUND.position));  

        collector.setDefaultCommand(new RunCommand(() -> collector.stow(), collector));
    }

    private void configureBindings() {

        // SmartDashboard.putData("Elevator Ground", new InstantCommand(() -> elevator.setPositionRevolutions(ElevatorConstants.FLOOR.GROUND.position)));
        // SmartDashboard.putData("Elevator Trough", new ScoreCoral(elevator, corraler, ElevatorConstants.FLOOR.TROUGH.position, ElevatorConstants.FLOOR.GROUND.position));
        // SmartDashboard.putData("Elevator Level 2", new ScoreCoral(elevator, corraler, ElevatorConstants.FLOOR.LEVEL_2.position, ElevatorConstants.FLOOR.GROUND.position));
        // SmartDashboard.putData("Elevator Level 3", new ScoreCoral(elevator, corraler, ElevatorConstants.FLOOR.LEVEL_3.position, ElevatorConstants.FLOOR.GROUND.position));
        // SmartDashboard.putData("Elevator Level 4", new ScoreCoral(elevator, corraler, ElevatorConstants.FLOOR.LEVEL_4.position, ElevatorConstants.FLOOR.GROUND.position));

        SmartDashboard.putData("Outtake", new RunCommand(() -> corraler.outtakeCoral()).withTimeout(1).andThen(new InstantCommand(() -> corraler.stopCoralMotor())));
        // joystick.pov(0).whileTrue(drivetrain.applyRequest(() ->
        //     forwardStraight.withVelocityX(0.5).withVelocityY(0))
        // );
        
        // joystick.pov(180).whileTrue(drivetrain.applyRequest(() ->
        //     forwardStraight.withVelocityX(-0.5).withVelocityY(0))
        // );

        joystick.start().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        joystick.leftBumper().onTrue(new OuttakeCommand(corraler));

        //joystick.rightBumper().onTrue(new InstantCommand( () -> elevator.alterMech(armLength.getValue(), wristAngle.getValue())));

        // joystick.x().onTrue(new ScoreCoral(elevator, corraler, ElevatorConstants.FLOOR.TROUGH.position, ElevatorConstants.FLOOR.GROUND.position).withName("score trough"));
        // joystick.a().onTrue(new ScoreCoral(elevator, corraler, ElevatorConstants.FLOOR.LEVEL_2.position, ElevatorConstants.FLOOR.GROUND.position).withName("score L2"));
        // joystick.b().onTrue(new ScoreCoral(elevator, corraler, ElevatorConstants.FLOOR.LEVEL_3.position, ElevatorConstants.FLOOR.GROUND.position).withName("score L3"));
        // joystick.y().onTrue(new ScoreCoral(elevator, corraler, ElevatorConstants.FLOOR.LEVEL_4.position, ElevatorConstants.FLOOR.GROUND.position).withName("score L4"));

        //joystick.rightBumper().whileTrue(new AlignToTarget(drivetrain, drivetrain.m_photonCameraWrapper, 18, Preferences.alignAdj));

        SmartDashboard.putData("Climber Test", new RunCommand(() -> climber.setServoPosition(0.5)));

        joystick.povUp().onTrue(new InstantCommand(() -> climber.setSpeed(Preferences.climberUpSpeed)))
                    .onFalse(new InstantCommand(() -> climber.setSpeed(0)));
        joystick.povDown().onTrue(new InstantCommand(() -> climber.setSpeed(Preferences.climberDownSpeed)))
                    .onFalse(new InstantCommand(() -> climber.setSpeed(0)));
        joystick.povLeft().whileTrue(new InstantCommand(() -> climber.setSpeed(0)));
        

        //score coral
        // joystick.a().whileTrue(new AutoScoreCoralGroup(drivetrain, elevator, corraler, drivetrain.m_photonCameraWrapper, 
        //         Preferences.coralXDriveOffset, ()-> joystick.getLeftY(), joystick.rightTrigger())
        // );

        joystick.a().whileTrue(new SemiAutoScoreCoralGroup(drivetrain, elevator, corraler, drivetrain.m_photonCameraWrapper, 
                Preferences.coralXDriveOffset, ()-> joystick.getLeftY(), () -> joystick.getLeftX(), joystick.rightTrigger(), joystick.leftTrigger())
        ).
        onFalse(
            new ToSetpoint(elevator, ElevatorConstants.FLOOR.GROUND.position)
            .finallyDo(() -> CoralState.getInstance().setCoralTarget(CoralTarget.NONE))
            );

        //joystick.b().whileTrue(new IntakeAlgae(drivetrain, m_allianceState.getReefTags(), Preferences.alignAdj.getValue(), () -> joystick.getLeftY(), () -> joystick.getLeftX(), elevator, collector, ElevatorConstants.ALGAE.HIGH_REEF.height));
        joystick.x().whileTrue(
            new ToSetpoint(elevator, ElevatorConstants.ALGAE.PROCESSOR.height).withTimeout(2).alongWith(
                new RunCommand(() -> collector.setWristPosition(AlgaeCollectorConstants.WRIST.PROCESS.angle)).until(() -> collector.isWristAtPosition()).withTimeout(2)
            ).andThen(new WaitUntilCommand(joystick.rightTrigger()).andThen(
                new RunCommand(() -> collector.outtakeAlgae()).withTimeout( .5))
            )
        ).onFalse(
            new ToSetpoint(elevator, ElevatorConstants.FLOOR.GROUND.position).alongWith(
                new InstantCommand(() -> collector.setWristPosition(AlgaeCollectorConstants.WRIST.STOW.angle))).alongWith(
                new InstantCommand(() -> collector.stopAlgaeMotor()))
        );

        joystick.b().onTrue(
            new ElevatorToAlgaePreset(elevator).alongWith(
                new RunCommand(() -> collector.setToIntakePosition()).alongWith(
                    new AlignThenDrive(drivetrain, m_PhotonCameraWrapper, AllianceState.getInstance().getReefTags(), 1, 0.5, CameraConstants.getAlgaeYawOffsetDegreesRight(0.5),() -> joystick.getLeftY(),() ->  joystick.getLeftX()).alongWith(
                        new RunCommand(() -> collector.intakeAlgae())
                    )
                )
            )
        ).onFalse(

            new ToSetpoint(elevator, ElevatorConstants.FLOOR.GROUND.position).alongWith(
                new RunCommand(() -> collector.setWristPosition(AlgaeCollectorConstants.WRIST.STOW.angle))
            )
        );

        joystick.y().onTrue(
            new ToSetpoint(elevator, ElevatorConstants.ALGAE.BARGE.height).alongWith(
                new RunCommand(() -> collector.setWristPosition(AlgaeCollectorConstants.WRIST.BARGE.angle)).until(() -> collector.isWristAtPosition())
            ).andThen(new WaitUntilCommand(joystick.rightTrigger()).andThen(
                new RunCommand(() -> collector.outtakeAlgae()).withTimeout(.5))
            )
        ).onFalse(
            new ToSetpoint(elevator, ElevatorConstants.FLOOR.GROUND.position).alongWith(
                new InstantCommand(() -> collector.setWristPosition(AlgaeCollectorConstants.WRIST.STOW.angle))).alongWith(
                new InstantCommand(() -> collector.stopAlgaeMotor()))
        );

        joystick.back().onTrue(new InstantCommand(() -> CommandScheduler.getInstance().cancelAll())
            .andThen(new InstantCommand(() -> collector.stopAlgaeMotor()))
            .andThen(new InstantCommand(() -> corraler.stopCoralMotor()))
            .andThen(new InstantCommand(() -> elevator.setPositionRevolutions(ElevatorConstants.FLOOR.GROUND.position)))
            .andThen(new InstantCommand(() -> collector.stow()))
        );

                                         
    }

    private void configureTestBindings() {
        joystick.a().whileTrue(new RotateToHeading(drivetrain, m_PhotonCameraWrapper, () -> Preferences.rotationTestTarget.get()));
        joystick.y().whileTrue(new AlignSideToSide(drivetrain, m_PhotonCameraWrapper));
        joystick.x().whileTrue(new DriveIntoTarget(drivetrain, m_PhotonCameraWrapper));
        joystick.b().whileTrue(new AlignToReefTags(drivetrain, m_PhotonCameraWrapper, ()-> joystick.getLeftY(), () -> joystick.getLeftX()));

        joystick.start().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
    
    }

    public Command getAutonomousCommand() {
        /* First put the drivetrain into auto run mode, then run the auto */
        return autoChooser.getSelected();
    }

   
}
