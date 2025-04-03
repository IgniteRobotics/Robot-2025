// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import static edu.wpi.first.units.Units.*;


/** Add your docs here. */
public class DriveCommandConstants {

    public static final double ROTATION_P = 0.1;
    public static final double ROTATION_D = 0.00005;

    public static final double DRIVE_X_P = 2.6;
    public static final double DRIVE_X_D = 0.015;

    public static final double DRIVE_Y_P = 2.6;
    public static final double DRIVE_Y_D = 0.000001;

    public static final double DRIVE_Y_TOLERANCE = 0.25;
    public static final double DRIVE_X_TOLERANCE = 0.1;
    public static final double ROTATION_TOLERANCE = 3.5;

    public static final double REEF_PUSH_DRIVE_X = 0.75;

    //max XY in m/
    public static final double SLOW_DRIVE_XY_FACTOR = 1.25;
    //max rotations in radians/s
    public static final double SLOW_DRIVE_ROTATION_FACTOR = RotationsPerSecond.of(0.25).in(RadiansPerSecond);;


}
