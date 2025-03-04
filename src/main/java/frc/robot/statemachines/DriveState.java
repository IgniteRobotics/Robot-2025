// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.statemachines;

import java.util.HashMap;
import java.util.Map;

import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.generated.TunerConstants;
import frc.zones.Grid;
import frc.zones.Zone;
import static edu.wpi.first.units.Units.MetersPerSecond;

@Logged
public class DriveState {

    private AllianceState allianceState = AllianceState.getInstance();

    private static DriveState single_instance = null;

    private final double blockWidth = 0.25;

    private static Pose2d robotPose2d;

    Map<String, PhotonPipelineResult> cameraResults = new HashMap<>(){};

    private DriveState() {

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

    //**********ZONE***********//
    public Zone[][] getGrid(){
        if(allianceState.getAlliance() == Alliance.Red){
            return Grid.RED_GRID;
        }
        else return Grid.BLUE_GRID;
    }

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

    @Logged(name = "Zone", importance = Importance.CRITICAL)
    public String getZoneName(){
        if(getZone() == null) return "currentZone is nonexistent";
        else return getZone().name;
    }

    @Logged(name = "Max Speed", importance = Importance.CRITICAL)
    public double getMaxSpeed(){
        if(getZone() == null){
            return TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
        }
        else return getZone().maxSpeed.doubleValue();
    }

    @Logged(name = "Max Rotation", importance = Importance.CRITICAL)
    public double getMaxRotation(){
        if(getZone() == null){
            return TunerConstants.MAX_ANGULAR_SPEED;
        }
        else return getZone().maxRotation.doubleValue();
    }

    //**********Vision***********//
    public void setLatestPhotonVisionResult(String camera, PhotonPipelineResult newResult){
        cameraResults.put(camera, newResult);
    }

    public PhotonPipelineResult getLatestPhotonVisionResult(String camera){
        if(cameraResults.containsKey(camera))return cameraResults.get(camera);
        else return null;
    }


}
