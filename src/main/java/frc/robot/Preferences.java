// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.PreferenceTypes.DoublePreference;
import frc.robot.subsystems.climber.ClimberConstants;
import frc.robot.subsystems.Elevator.ElevatorConstants;
import frc.robot.subsystems.EndEffector.EndEffectorConstants;

/** Add your docs here. */
public class Preferences {

    // ********** Drive **********
    public static DoublePreference alignDistanceAdjustment = new DoublePreference("alignCommand/distanceAdjustment");

    public static DoublePreference alignRotKP = new DoublePreference("alignCommand/rotation/kP", 0);
    public static DoublePreference alignRotKD = new DoublePreference("alignCommand/rotation/kD", 0);
    public static DoublePreference alignDriveXKP = new DoublePreference("alignCommand/drive/x/kP", 0);
    public static DoublePreference alignDriveXKD = new DoublePreference("alignCommand/drive/x/kD", 0);
    public static DoublePreference alignDriveYKP = new DoublePreference("alignCommand/drive/y/kP", 0);
    public static DoublePreference alignDriveYKD = new DoublePreference("alignCommand/drive/y/KD", 0);

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
    
    // ********** End Effector **********
    public static DoublePreference endEffectorCoralIntakePower = new DoublePreference("EndEffector/Coral/IntakePower", EndEffectorConstants.INTAKE_CORAL_POWER);
    public static DoublePreference endEffectorCoralOuttakePower = new DoublePreference("EndEffector/Coral/OuttakePower", EndEffectorConstants.OUTTAKE_CORAL_POWER);

    public static DoublePreference endEffectorStowPositionNoAlgae = new DoublePreference("EndEffector/Wrist/Stow Position/No Algae", EndEffectorConstants.STOW_POSITION_NO_ALGAE);
    public static DoublePreference endEffectorStowPositionWithAlgae = new DoublePreference("EndEffector/Wrist/Stow Position/WithAlgae", EndEffectorConstants.STOW_POSITION_WITH_ALGAE);
    
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
    public static DoublePreference climberDownSpeed = new DoublePreference("Clibmer/Down Power", 0.1);


    public static DoublePreference outtakeDelay = new DoublePreference("EndEffector/Outtake Delay", 0.5);
}
