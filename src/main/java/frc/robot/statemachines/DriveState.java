// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.statemachines;

import java.util.HashMap;
import java.util.Map;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Preferences;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.CameraConstants;
import frc.zones.Grid;
import frc.zones.Zone;
import static edu.wpi.first.units.Units.MetersPerSecond;

@Logged
public class DriveState {

    private AllianceState allianceState = AllianceState.getInstance();

    private static DriveState single_instance = null;

    private final double blockWidth = 0.25;

    private static Pose2d robotPose2d;

    private static Pose2d targetPose2d;

    private static double robotYaw;

    private static double maxSpeed;
    
    private static double maxAngularRate;

    Map<PhotonCamera, PhotonPipelineResult> cameraResults = new HashMap<>(){};

    //use pose estimators
    private boolean useOuttakeLeftEstimation = true;
    private boolean useOuttakeRightEstimation = true;
    private boolean useIntakeEstimation = true;

    private DriveState() {
        maxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
        maxAngularRate = TunerConstants.MAX_ANGULAR_SPEED;
    }

    public static synchronized DriveState getInstance()
    {
        if (single_instance == null)
            single_instance = new DriveState();
        
        return single_instance;
    }

    //**********PATH***********//
    public synchronized void setPose2d(Pose2d newPose){
        robotPose2d = newPose;
    }

    public Pose2d getPose2d(){
        return robotPose2d;
    }

    public Pose2d getTargetPose2d(){
        return targetPose2d;
    }

    public synchronized void setTargetPose2d(Pose2d newPose){
        targetPose2d = newPose;
    }

    public synchronized void setYaw(double yaw){
        robotYaw = yaw;
    }

    public double getYaw(){
        return robotYaw;
    }

    //**********ZONE***********//
    public Zone[][] getGrid(){
        if(allianceState.getAlliance() == Alliance.Red){
            return Grid.RED_GRID;
        }
        else return Grid.BLUE_GRID;
    }
    /* 
    @NotLogged
    public Zone getZone(){
        if(robotPose2d == null 
        || (int)(robotPose2d.getX()/blockWidth) >= Grid.xLength || (int)(robotPose2d.getY()/blockWidth) >= Grid.yLength
        || robotPose2d.getX() < 0 || robotPose2d.getY() < 0){
            return null;
        }
        else{
            return getGrid()[(int)(robotPose2d.getX()/blockWidth)][ (int)(robotPose2d.getY()/blockWidth)];
        }
    }
    */

    /* 
    @Logged(name = "Zone", importance = Importance.CRITICAL)
    public String getZoneName(){
        if(getZone() == null) return "currentZone is nonexistent";
        else return getZone().name;
    }
    */

    @Logged(name = "Max Speed", importance = Importance.CRITICAL)
    public double getMaxSpeed(){
        /* 
        if(getZone() == null){
            return TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
        }
        else return getZone().maxSpeed.doubleValue();
        */
        return maxSpeed;
    }

    @Logged(name = "Max Rotation", importance = Importance.CRITICAL)
    public double getMaxRotation(){
        /* 
        if(getZone() == null){
            return TunerConstants.MAX_ANGULAR_SPEED;
        }
        else return getZone().maxRotation.doubleValue();
        */
        return maxAngularRate;
    }

    public void nerf(){
        maxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond)*Preferences.nerfFactor.getValue();
        maxAngularRate = TunerConstants.MAX_ANGULAR_SPEED*Preferences.nerfFactor.getValue();
    }

    public void unNerf(){
        maxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
        maxAngularRate = TunerConstants.MAX_ANGULAR_SPEED;
    }

    //**********Vision***********//
    public void lockToCamera(PhotonCamera camera){
        if(camera == CameraConstants.photonCameraOuttakeLeft){
            useIntakeEstimation= false;
            useOuttakeLeftEstimation = true;
            useOuttakeRightEstimation = false;
        }

        else if(camera == CameraConstants.photonCameraOuttakeRight){
            useIntakeEstimation = false;
            useOuttakeLeftEstimation = false;
            useOuttakeRightEstimation = true;
        }

        else {
            useIntakeEstimation = true;
            useOuttakeLeftEstimation = false;
            useOuttakeRightEstimation = false;
        }
    }

    public void unlockCameras(){
        useOuttakeLeftEstimation = true;
        useOuttakeRightEstimation = true;
        useIntakeEstimation = true;
    }

    public boolean useLeftOuttakeCamera(){
        return useOuttakeLeftEstimation;
    }

    public boolean useRightOuttakeCamera(){
        return useOuttakeRightEstimation;
    }

    public boolean useIntakeCamera(){
        return useIntakeEstimation;
    }
    
    public void setLatestPhotonVisionResult(PhotonCamera camera, PhotonPipelineResult newResult){
        cameraResults.put(camera, newResult);
    }

    public PhotonPipelineResult getLatestPhotonVisionResult(PhotonCamera camera){
        return cameraResults.get(camera);
    }

    public void nullify(PhotonCamera camera){
        cameraResults.put(camera, null);
    }

    public boolean hasPhotonVisionResult(PhotonCamera camera){
        return cameraResults.containsKey(camera) && cameraResults.get(camera) != null;
    }

    public static final double CUSTOM_FIELD_LENGTH = 9.144;
    public static final double CUSTOM_FIELD_WIDTH = 7.3152;

    public static final double NEED_SPACE = 1;

    public double customConstraintVelocityX(double initial){
        if(initial < 0 && getPose2d().getX() < 0){
            return 0;
        }
        else if(initial < 0 && getPose2d().getX() < NEED_SPACE){
            return initial * 0.35;
        }
        else if(initial > 0 && getPose2d().getX() > CUSTOM_FIELD_LENGTH){
            return 0;
        }
        else if(initial > 0 && getPose2d().getX() > CUSTOM_FIELD_LENGTH - NEED_SPACE){
            return initial * 0.35;
        }
        else return initial;
    }

    public double customConstraintVelocityY(double initial){
        if(initial < 0 && getPose2d().getY() < 0){
            return 0;
        }
        else if(initial < 0 && getPose2d().getY() < NEED_SPACE){
            return initial * 0.35;
        }
        else if(initial > 0 && getPose2d().getY() > CUSTOM_FIELD_WIDTH){
            return 0;
        }
        else if(initial > 0 && getPose2d().getY() > CUSTOM_FIELD_WIDTH - NEED_SPACE){
            return initial * 0.35;
        }
        else return initial;
    }


}
