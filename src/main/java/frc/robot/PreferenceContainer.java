// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Preferences.DoublePreference;

/** Add your docs here. */
public class PreferenceContainer {
    public DoublePreference elevatorkP = new DoublePreference("Elevator/kP", 0);
    public DoublePreference elevatorkI = new DoublePreference("Elevator/kI", 0);
    public DoublePreference elevatorkD = new DoublePreference("Elevator/kD", 0);
    public DoublePreference elevatorPosition = new DoublePreference("Elevator/Position", 0);

    
}
