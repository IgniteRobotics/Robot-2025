// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.PreferenceTypes.DoublePreference;
import frc.robot.subsystems.climber.ClimberConstants;
import frc.robot.subsystems.coral.CorralerConstants;
import frc.robot.subsystems.Elevator.ElevatorConstants;
import frc.robot.subsystems.algae.AlgaeCollector;
import frc.robot.subsystems.algae.AlgaeCollectorConstants;

/** Add your docs here. */
public class Preferences {

    // ********** Drive *********//

    
    public static DoublePreference alignRotKP = new DoublePreference("alignCommand/rotation/kP", 0.5);
    public static DoublePreference alignRotKD = new DoublePreference("alignCommand/rotation/kD", 0);
    public static DoublePreference alignDriveYKP = new DoublePreference("alignCommand/driveY/kP", 0.5);
    public static DoublePreference alignDriveYKD = new DoublePreference("alignCommand/driveY/kD", 0.3);
    public static DoublePreference alignDriveXKP = new DoublePreference("alignCommand/driveX/kP", 0.5);
    public static DoublePreference alignDriveXKD = new DoublePreference("alignCommand/driveX/kD", 0.3);
    public static DoublePreference alignAdj = new DoublePreference("alignCommand/adjustment", 0);

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
    
    public static DoublePreference elevatorAutonHeight = new DoublePreference("Elevator/Auton Height", 3.4);
    // ********** Corraler ********** 
    public static DoublePreference coralIntakePower = new DoublePreference("Coraller/IntakePower", CorralerConstants.INTAKE_CORAL_POWER);
    public static DoublePreference coralOuttakePower = new DoublePreference("Coraller/OuttakePower", CorralerConstants.OUTTAKE_CORAL_POWER);
    public static DoublePreference coralXDriveOffset = new DoublePreference("Coraller/XDriveOffset", 0.02);

    // ********** Algae Collector****
    public static DoublePreference algaeIntakePower = new DoublePreference("AlgaeCollector/IntakePower", AlgaeCollectorConstants.INTAKE_ALGAE_POWER);
    public static DoublePreference algaeOuttakePower = new DoublePreference("AlgaeCollector/OuttakePower", AlgaeCollectorConstants.OUTTAKE_ALGAE_POWER);
    public static DoublePreference algaeHoldPower = new DoublePreference("AlgaeCollector/HopperPower", AlgaeCollectorConstants.ALGAE_HOLD_POWER);

    public static DoublePreference collectorStowPosition = new DoublePreference("AlgaeCollector/Wrist/Stow Position", AlgaeCollectorConstants.WRIST.STOW.angle);
    public static DoublePreference collectorWristPosition = new DoublePreference("AlgaeCollector/Wrist/Position",0);

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

    public static DoublePreference climberPos1 = new DoublePreference("Climber/Position 1", 1);
    public static DoublePreference climberPos2 = new DoublePreference("Climber/Position 2", 2);

    
}
