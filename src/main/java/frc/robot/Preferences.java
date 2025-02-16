// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.PreferenceTypes.DoublePreference;
import frc.robot.subsystems.Elevator.ElevatorConstants;

/** Add your docs here. */
public class Preferences {

    public static DoublePreference alignDistanceAdjustment = new DoublePreference("alignCommand/distanceAdjustment");

    public static DoublePreference alignRotKP = new DoublePreference("alignCommand/rotation/kP", 0);
    public static DoublePreference alignRotKD = new DoublePreference("alignCommand/rotation/kD", 0);
    public static DoublePreference alignDriveXKP = new DoublePreference("alignCommand/drive/x/kP", 0);
    public static DoublePreference alignDriveXKD = new DoublePreference("alignCommand/drive/x/kD", 0);
    public static DoublePreference alignDriveYKP = new DoublePreference("alignCommand/drive/y/kP", 0);
    public static DoublePreference alignDriveYKD = new DoublePreference("alignCommand/drive/y/KD", 0);

    public static DoublePreference elevatorkP = new DoublePreference("Elevator/kP", ElevatorConstants.ELEVATOR_kP);
    public static DoublePreference elevatorkI = new DoublePreference("Elevator/kI", ElevatorConstants.ELEVATOR_kI);
    public static DoublePreference elevatorkD = new DoublePreference("Elevator/kD", ElevatorConstants.ELEVATOR_kD);
    public static DoublePreference elevatorkG = new DoublePreference("Elevator/kG", ElevatorConstants.ELEVATOR_kG);
    public static DoublePreference elevatorPosition = new DoublePreference("Elevator/Position", 0);



}
