// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.statemachines;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.subsystems.drive.CameraConstants;

public class AllianceState {

    private static AllianceState single_instance = null;

    

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

}
