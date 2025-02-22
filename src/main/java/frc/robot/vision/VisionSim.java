// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.vision;

import org.photonvision.PhotonCamera;
import org.photonvision.estimation.TargetModel;
import org.photonvision.proto.Photon;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.simulation.VisionTargetSim;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.commands.drive.AlignToTarget;
import frc.robot.subsystems.drive.CameraConstants;

/** Add your docs here. */
public class VisionSim {
    private VisionSystemSim m_visionSystemSim;
    private Pose3d robotPoseMeters = new Pose3d();

    private PhotonCameraSim m_cameraSimFL;
    private PhotonCameraSim m_cameraSimFR;
    private PhotonCameraSim m_cameraSimBack;

    private Transform3d m_robotToCameraFL;
    private Transform3d m_robotToCameraFR;
    private Transform3d m_robotToCameraBack;


    public VisionSim() {
        //creates a new vision system sim in NetworkTables (simulated world)
        m_visionSystemSim = new VisionSystemSim("Vision System");
        AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);
        TargetModel targetModel = TargetModel.kAprilTag36h11;

        m_visionSystemSim.addAprilTags(aprilTagFieldLayout);

        SimCameraProperties FLProp = new SimCameraProperties();
        FLProp.setCalibration(1920, 1080, Rotation2d.fromDegrees(100));
        FLProp.setCalibError(0.25, 0.08);
        FLProp.setFPS(20);
        FLProp.setAvgLatencyMs(35);
        FLProp.setLatencyStdDevMs(5);

        SimCameraProperties FRProp = new SimCameraProperties();
        FRProp.setCalibration(1920, 1080, Rotation2d.fromDegrees(100));
        FRProp.setCalibError(0.25, 0.08);
        FRProp.setFPS(20);
        FRProp.setAvgLatencyMs(35);
        FRProp.setLatencyStdDevMs(5);

        SimCameraProperties BackProp = new SimCameraProperties();
        BackProp.setCalibration(1920, 1080, Rotation2d.fromDegrees(100));
        BackProp.setCalibError(0.25, 0.08);
        BackProp.setFPS(20);
        BackProp.setAvgLatencyMs(35);
        BackProp.setLatencyStdDevMs(5);

        PhotonCamera cameraFL = new PhotonCamera("FL Camera");
        m_cameraSimFL = new PhotonCameraSim(cameraFL, FLProp);
        m_robotToCameraFL = CameraConstants.photonCameraTransformOuttakeLeft;


        PhotonCamera cameraFR = new PhotonCamera("FR Camera");
        m_cameraSimFR = new PhotonCameraSim(cameraFR, FRProp);
        m_robotToCameraFR = CameraConstants.photonCameraTransformOuttakeRight;


        PhotonCamera cameraBack = new PhotonCamera("Back Camera");
        m_cameraSimBack = new PhotonCameraSim(cameraBack, BackProp);
        m_robotToCameraBack = CameraConstants.photonCameraTransformIntake;     
    }

    public void setupCameras() {
        m_visionSystemSim.addCamera(m_cameraSimFL, m_robotToCameraFL);
        m_visionSystemSim.addCamera(m_cameraSimFR, m_robotToCameraFR);
        m_visionSystemSim.addCamera(m_cameraSimBack, m_robotToCameraBack);
    }
}