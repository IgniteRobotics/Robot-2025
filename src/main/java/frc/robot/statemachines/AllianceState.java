// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.statemachines;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.subsystems.drive.CameraConstants;

public class AllianceState {

    private static AllianceState single_instance = null;

    public static final double BLUE_REEF_X = 4.471;
    public static final double BLUE_REEF_Y = 4.055;

    public static final double RED_REEF_X = 17.55 - 4.471;
    public static final double RED_REEF_Y = 4.055;

    

    private AllianceState() {
        
    }

    public static synchronized AllianceState getInstance()
    {
        if (single_instance == null)
            single_instance = new AllianceState();
        
        return single_instance;
    }

    // public void setAlliance(Alliance newAlliance){
    //     alliance = newAlliance;
    // }

    public Alliance getAlliance(){
        return DriverStation.isDSAttached() ? DriverStation.getAlliance().get() : Alliance.Blue;
    }

    public int[] getReefTags(){
        if(getAlliance() == Alliance.Red){
            return CameraConstants.RED_REEF_TAGS;
        }
        else return CameraConstants.BLUE_REEF_TAGS;
    }

    public int[] getBargeTags(){
        if(getAlliance() == Alliance.Red){
            return CameraConstants.RED_BARGE_TAGS;
        }
        else return CameraConstants.BLUE_BARGE_TAGS;
    }

    public int[] getProcessorTags(){
        if(getAlliance() == Alliance.Red){
            return CameraConstants.RED_PROCESSOR_TAGS;
        }
        else return CameraConstants.BLUE_PROCESSOR_TAGS;
    }   

    public int[] getHumanPlayerTags(){
        if(getAlliance() == Alliance.Red){
            return CameraConstants.RED_HUMAN_PLAYER_TAGS;
        } else {
            return CameraConstants.BLUE_HUMAN_PLAYER_TAGS;
        }
    }

    public double getHeadingToReef(Pose2d pose2d){
        double deltaX;
        double deltaY;

        if(getAlliance() == Alliance.Red){
            deltaX = RED_REEF_X - pose2d.getX();
            deltaY = RED_REEF_Y - pose2d.getY(); 
        }
        else{
            deltaX = BLUE_REEF_X - pose2d.getX();
            deltaY = BLUE_REEF_Y - pose2d.getY();
        }

        return Math.atan2(deltaY, deltaX);
    }

}