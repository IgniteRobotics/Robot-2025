// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Preferences.DoublePreference;
import frc.robot.subsystems.Elevator.ElevatorConstants;

/** Add your docs here. */
public class PreferenceContainer {
    public DoublePreference elevatorkP = new DoublePreference("Elevator/kP", ElevatorConstants.ELEVATOR_kP);
    public DoublePreference elevatorkI = new DoublePreference("Elevator/kI", ElevatorConstants.ELEVATOR_kI);
    public DoublePreference elevatorkD = new DoublePreference("Elevator/kD", ElevatorConstants.ELEVATOR_kD);
    public DoublePreference elevatorkG = new DoublePreference("Elevator/kG", ElevatorConstants.ELEVATOR_kG);
    public DoublePreference elevatorPosition = new DoublePreference("Elevator/Position", 0);

    
}
