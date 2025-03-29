// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.test;

import java.util.Optional;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.Preferences;
import frc.robot.statemachines.AllianceState;
import frc.robot.statemachines.CoralState;
import frc.robot.subsystems.drive.CameraConstants;
import frc.robot.subsystems.drive.PhotonCameraWrapper;
import frc.robot.subsystems.drive.PhotonCameraWrapper.TargetInfo;

/** Add your docs here. */
public class VisionTest {

    public static void VisionValueTest(PhotonCameraWrapper m_pcw) {
        Optional<TargetInfo> targeting = m_pcw.seekTargets(AllianceState.getInstance().getReefTags(), CameraConstants.photonCameraOuttakeLeft);
        
        System.out.println("getX() from transform3d: " + targeting.get().getTransform3d().getX());
        System.out.println("getY() from transform3d: " + targeting.get().getTransform3d().getY());
        
        PIDController sampleController = new PIDController(1, 0, 0);
        System.out.println("X velocity determined: " + sampleController.calculate(targeting.get().getTransform3d().getX()));
        System.out.println("Y velocity determined: " + sampleController.calculate(targeting.get().getTransform3d().getY()));
    }

}
