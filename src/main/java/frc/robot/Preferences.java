// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.PreferenceTypes.DoublePreference;
import frc.robot.commands.drive.DriveCommandConstants;
import frc.robot.subsystems.climber.ClimberConstants;
import frc.robot.subsystems.drive.CameraConstants;
import frc.robot.subsystems.Elevator.ElevatorConstants;
import frc.robot.subsystems.algae.AlgaeCollector;
import frc.robot.subsystems.algae.AlgaeCollectorConstants;

/** Add your docs here. */
public class Preferences {

    // ********** Drive *********//
    public static DoublePreference nerfFactor = new DoublePreference("Nerf Factor", 0.35);
    public static DoublePreference alignTimeout = new DoublePreference("Alignment Timeout", 1.5);

    public static DoublePreference alignYOffsetRight = new DoublePreference("alignCommand/driveY/offset/right", CameraConstants.Y_RIGHT_CORAL_OFFSET_METERS);
    public static DoublePreference alignYOffsetLeft = new DoublePreference("alignCommand/driveY/offset/left", CameraConstants.Y_LEFT_CORAL_OFFSET_METERS);
    public static DoublePreference alignXOffset = new DoublePreference("alignCommand/driveX/offset", CameraConstants.X_OFFSET_METERS);
    
    public static DoublePreference alignRotKP = new DoublePreference("alignCommand/rotation/kP", DriveCommandConstants.ROTATION_P);
    public static DoublePreference alignRotKD = new DoublePreference("alignCommand/rotation/kD", DriveCommandConstants.ROTATION_D);
    public static DoublePreference alignRotKI = new DoublePreference("alignCommand/rotation/kI", 0);
    
    public static DoublePreference alignDriveYKP = new DoublePreference("alignCommand/driveY/kP", DriveCommandConstants.DRIVE_Y_P);
    public static DoublePreference alignDriveYKD = new DoublePreference("alignCommand/driveY/kD", DriveCommandConstants.DRIVE_Y_D);
    public static DoublePreference alignDriveYKI = new DoublePreference("alignCommand/driveY/kI", 0);
    
    public static DoublePreference alignDriveXKP = new DoublePreference("alignCommand/driveX/kP", DriveCommandConstants.DRIVE_X_P);
    public static DoublePreference alignDriveXKD = new DoublePreference("alignCommand/driveX/kD", DriveCommandConstants.DRIVE_X_D);
    public static DoublePreference alignDriveXKI = new DoublePreference("alignCommand/driveX/kI",0);

    public static DoublePreference profiledAlignRotKP = new DoublePreference("profiledAlignCommand/rotation/kP", DriveCommandConstants.ROTATION_P);
    public static DoublePreference profiledAlignRotKD = new DoublePreference("profiledAlignCommand/rotation/kD", DriveCommandConstants.ROTATION_D);
    public static DoublePreference profiledAlignRotKI = new DoublePreference("profiledAlignCommand/rotation/kI", 0);
    public static DoublePreference profiledAlignRotMaxVel = new DoublePreference("profiledAlignCommand/rotation/maxVel", 45);
    public static DoublePreference profiledAlignRotMaxAcc = new DoublePreference("profiledAlignCommand/rotation/maxAcc", 90);    

    public static DoublePreference profiledDriveYKP = new DoublePreference("profiledAlignCommand/driveY/kP", DriveCommandConstants.DRIVE_Y_P);
    public static DoublePreference profiledDriveYKD = new DoublePreference("profiledAlignCommand/driveY/kD", DriveCommandConstants.DRIVE_Y_D);
    public static DoublePreference profiledDriveYKI = new DoublePreference("profiledAlignCommand/driveY/kI", 0);
    public static DoublePreference profiledDriveYMaxVel = new DoublePreference("profiledAlignCommand/driveY/maxVel", 8);
    public static DoublePreference profiledDriveYMaxAcc = new DoublePreference("profiledAlignCommand/driveY/maxAcc", 16);

    public static DoublePreference profiledDriveXKP = new DoublePreference("profiledAlignCommand/driveX/kP", DriveCommandConstants.DRIVE_X_P);
    public static DoublePreference profiledDriveXKD = new DoublePreference("profiledAlignCommand/driveX/kD", DriveCommandConstants.DRIVE_X_D);
    public static DoublePreference profiledDriveXKI = new DoublePreference("profiledAlignCommand/driveX/kI", 0);
    public static DoublePreference profiledDriveXMaxVel = new DoublePreference("profiledAlignCommand/driveX/maxVel", 6);
    public static DoublePreference profiledDriveXMaxAcc = new DoublePreference("profiledAlignCommand/driveX/maxAcc", 12);

    public static DoublePreference alignAdj = new DoublePreference("alignCommand/adjustment", 0);
    public static DoublePreference rotationTolerancePreference = new DoublePreference("alignCommand/rotationTolerance", DriveCommandConstants.ROTATION_TOLERANCE);
    public static DoublePreference rotationTestTarget = new DoublePreference("alignCommand/rotationTarget", 0);
    public static DoublePreference yAlignTolerancePreference = new DoublePreference("alignCommand/yAlignTolerance", DriveCommandConstants.DRIVE_Y_TOLERANCE);
    public static DoublePreference xAlignTolerancePreference = new DoublePreference("alignCommand/xAlignTolerance", DriveCommandConstants.DRIVE_X_TOLERANCE);
    public static DoublePreference reefPushAgainstPreference = new DoublePreference("alignCommang/reefPushMPS", DriveCommandConstants.REEF_PUSH_DRIVE_X);

    public static DoublePreference xySlowLimitPreference = new DoublePreference("drive/xySlowLimitMPS", DriveCommandConstants.SLOW_DRIVE_XY_FACTOR);
    public static DoublePreference rotationSlowLimitPreference = new DoublePreference("drive/rotationSlowLimitRadPS", DriveCommandConstants.SLOW_DRIVE_ROTATION_FACTOR);
    public static DoublePreference autonYDrive = new DoublePreference("drive/yAutonDrive", -5);

    public static DoublePreference driveTestTurnAngle = new DoublePreference("drive/test/turnAngle", 60);
    public static DoublePreference driveTestRPS = new DoublePreference("drive/test/RPS", 2);
    public static DoublePreference driveTestSpeed = new DoublePreference("drive/test/speed", 2);
    
    public static DoublePreference maxAlignDriveVelocity = new DoublePreference("alignCommand/driveVelocity", 2.5);
    

    // ********** Elevator **********
    public static DoublePreference elevatorkV = new DoublePreference("Elevator/kV", ElevatorConstants.ELEVATOR_kV);
    public static DoublePreference elevatorkP = new DoublePreference("Elevator/kP", ElevatorConstants.ELEVATOR_kP);
    public static DoublePreference elevatorkI = new DoublePreference("Elevator/kI", ElevatorConstants.ELEVATOR_kI);
    public static DoublePreference elevatorkD = new DoublePreference("Elevator/kD", ElevatorConstants.ELEVATOR_kD);
    public static DoublePreference elevatorkG = new DoublePreference("Elevator/kG", ElevatorConstants.ELEVATOR_kG);
    public static DoublePreference elevatorkS = new DoublePreference("Elevator/kS", ElevatorConstants.ELEVATOR_kS);
    public static DoublePreference elevatorPosition = new DoublePreference("Elevator/Position", 0);

    public static DoublePreference elevatorMMJerk = new DoublePreference("Elevator/Motion Magic/Jerk", ElevatorConstants.ELEVATOR_MM_JERK);
    public static DoublePreference elevatorMMAccel = new DoublePreference("Elevator/Motion Magic/Acceleration", ElevatorConstants.ELEVATOR_MM_ACCEL);
    public static DoublePreference elevatorMMCruiseVelocity = new DoublePreference("Elevator/Motion Magic/Cruise Velocity", ElevatorConstants.ELEVATOR_MM_CRUISE_VELOCITY);

    public static DoublePreference elevatorTroughBumpPreference = new DoublePreference("Elevator/TroughBump", ElevatorConstants.FLOOR.TROUGH.position);
    public static DoublePreference elevatorLowAlgaeReefPreference = new DoublePreference("Elevator/ReefLowAlgaeHeight", ElevatorConstants.ALGAE.LOW_REEF.height);
    public static DoublePreference elevatorHighAlgaeReefPreference = new DoublePreference("Elevator/ReefHighAlgaeHeight", ElevatorConstants.ALGAE.HIGH_REEF.height);
    
    // ********** Corraler ********** 
    public static DoublePreference coralIntakePower = new DoublePreference("Coraller/IntakePower", AlgaeCollectorConstants.INTAKE_CORAL_POWER);
    public static DoublePreference coralOuttakePower = new DoublePreference("Coraller/OuttakePower", AlgaeCollectorConstants.OUTTAKE_CORAL_POWER);
    public static DoublePreference coralXDriveOffset = new DoublePreference("Coraller/XDriveOffset", 0.02);

    // ********** Algae Collector****
    public static DoublePreference algaeIntakePower = new DoublePreference("AlgaeCollector/IntakePower", AlgaeCollectorConstants.INTAKE_ALGAE_POWER);
    public static DoublePreference algaeOuttakePower = new DoublePreference("AlgaeCollector/OuttakePower", AlgaeCollectorConstants.OUTTAKE_ALGAE_POWER);
    public static DoublePreference algaeHoldPower = new DoublePreference("AlgaeCollector/HopperPower", AlgaeCollectorConstants.ALGAE_HOLD_POWER);
    public static DoublePreference algaeIntakeCurrentLimit = new DoublePreference("AlgaeCollector/IntakeCurrentLimit", AlgaeCollectorConstants.ALGAE_INTAKE_CURRENT_LIMIT);

    public static DoublePreference collectorStowPosition = new DoublePreference("AlgaeCollector/Wrist/Stow Position", AlgaeCollectorConstants.WRIST.STOW.angle);
    public static DoublePreference collectorIntakePosition = new DoublePreference("AlgaeCollector/Wrist/Intake Position", AlgaeCollectorConstants.WRIST.REEF.angle);

    public static DoublePreference collectorOuttakeDelay = new DoublePreference("AlgaeCollector/Outtake Delay", 0.5);

    // ********** Climber **********
    public static DoublePreference climberkV = new DoublePreference("Climber/kV", ClimberConstants.CLIMBER_kV);
    public static DoublePreference climberkP = new DoublePreference("Climber/kP", ClimberConstants.CLIMBER_kP);
    public static DoublePreference climberkI = new DoublePreference("Climber/kI", ClimberConstants.CLIMBER_kI);
    public static DoublePreference climberkD = new DoublePreference("Climber/kD", ClimberConstants.CLIMBER_kD);
    public static DoublePreference climberkG = new DoublePreference("Climber/kG", ClimberConstants.CLIMBER_kG);
    public static DoublePreference climberkS = new DoublePreference("Climber/kS", ClimberConstants.CLIMBER_kS);
    public static DoublePreference climberPosition = new DoublePreference("Climber/Position", 0);

    public static DoublePreference climberMMJerk = new DoublePreference("Climber/Motion Magic/Jerk", ClimberConstants.CLIMBER_MM_JERK);
    public static DoublePreference climberMMAccel = new DoublePreference("Climber/Motion Magic/Acceleration", ClimberConstants.CLIMBER_MM_ACCEL);
    public static DoublePreference climberMMCruiseVelocity = new DoublePreference("Climber/Motion Magic/Cruise Velocity", ClimberConstants.CLIMBER_MM_CRUISE_VELOCITY);

    public static DoublePreference climberUpSpeed = new DoublePreference("Climber/Up Power", 0.1);
    public static DoublePreference climberDownSpeed = new DoublePreference("Climber/Down Power", 0.1);

    public static DoublePreference servoPosition = new DoublePreference("Climber/Servo Position", 0);

    public static DoublePreference climberPos1 = new DoublePreference("Climber/Position Climb", -135.64);
    public static DoublePreference climberPos2 = new DoublePreference("Climber/Position Reach", 128.98);

    //********** auton **********
    public static DoublePreference autonDriveTimeoutPreference = new DoublePreference("auton/drive/timeout", 4);

    
}
