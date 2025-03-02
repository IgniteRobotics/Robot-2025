// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.tests;

import frc.robot.subsystems.drive.PhotonCameraWrapper;
import frc.robot.subsystems.drive.PhotonCameraWrapper.TargetInfo;

/** Add your docs here. */
public class VisionTests {
    private PhotonCameraWrapper m_cameraWrapper;

    public VisionTests() {
        // Add your constructor logic here
        m_cameraWrapper = new PhotonCameraWrapper();
    }

    public void testTargetCalculation() {
        // Add your test logic here
        // PhotonCameraWrapper.TargetInfo target = m_cameraWrapper.seekTarget(0);
        // System.out.println("Target: " + target);
        double[] tests = {-5, -1, 0, .5, 1, 5};

        for (double x : tests) {
            TargetInfo t = m_cameraWrapper.calculateTargetInfo(x, 10, 0, 0, "test");
            System.out.println("angle: " + x + " dist: " + 10 + " Turn: " + t.getYaw() + " Distance: " + t.getDistance());

        }

    }
}
