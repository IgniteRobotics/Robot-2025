// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.test;

import frc.robot.subsystems.drive.CameraConstants;

/** Add your docs here. */
public class PhotonAngleTests {

    private double[] distances =  {0, 0.2, 1, 2, 2.5, 5, 10};


    public PhotonAngleTests() {
        // Add your constructor implementation here
        for (double d : distances) {
            System.out.print("Input Distance: " + d);
            System.out.print(" Algae Distance: " + CameraConstants.getXOffsetMeters(d));
            System.out.print(" Alage Left Angle:  " + CameraConstants.getAlgaeYawOffsetDegreesLeft(d));
            System.out.print(" Alage Right Angle: " + CameraConstants.getAlgaeYawOffsetDegreesRight(d));
            System.out.print(" Coral Left Angle:  " + CameraConstants.getCorallYawOffsetDegreesLeft(d));
            System.out.println(" Coral Right Angle: " + CameraConstants.getCorallYawOffsetDegreesRight(d));
        }
            
    }

}
