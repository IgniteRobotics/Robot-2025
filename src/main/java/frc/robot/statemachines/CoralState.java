// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.statemachines;

import java.util.function.DoubleSupplier;

import org.photonvision.PhotonCamera;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.Preferences;
import frc.robot.subsystems.Elevator.ElevatorConstants;
import frc.robot.subsystems.algae.AlgaeCollectorConstants;
import frc.robot.subsystems.drive.CameraConstants;

@Logged
public class CoralState {

    private static CoralState single_instance = null;

    private CoralState() {

    }

    public static synchronized CoralState getInstance()
    {
        if (single_instance == null)
            single_instance = new CoralState();
        
        return single_instance;
    }

    private boolean hasCoral = false;

      //Desired Coral Targets
    public static enum CoralTarget {
        NONE("NONE"),
        TROUGH("TROUGH"),
        L2_LEFT("L2_LEFT"),
        L2_RIGHT("L2_RIGHT"),
        L3_LEFT("L3_LEFT"),
        L3_RIGHT("L3_RIGHT"),
        L4_LEFT("L4_LEFT"),
        L4_RIGHT("L4_RIGHT");

        public final String name;
        CoralTarget(String value){
            name = value;
            }
        }

    private CoralTarget coralTarget = CoralTarget.NONE;

    public void setCoralTarget(CoralTarget target){
        coralTarget = target;
    }
    
    public CoralTarget getCoralTarget(){
        return coralTarget;
    }

    @Logged(name = "Coral Target", importance = Importance.CRITICAL)
    public String getCoralTargetName(){
        return coralTarget.name;
    }

    @Logged(name = "CT_TROUGH", importance = Importance.CRITICAL)
    public boolean coralTarget_TROUGH(){
        return coralTarget == CoralTarget.TROUGH;
    }

    @Logged(name = "CT_L2_LEFT", importance = Importance.CRITICAL)
    public boolean coralTargetL2_LEFT(){
        return coralTarget == CoralTarget.L2_LEFT;
    }

    @Logged(name = "CT_L2_RIGHT", importance = Importance.CRITICAL)
    public boolean coralTargetL2_RIGHT(){
        return coralTarget == CoralTarget.L2_RIGHT;
    }

    @Logged(name = "CT_L3_LEFT", importance = Importance.CRITICAL)
    public boolean coralTargetL3_LEFT(){
        return coralTarget == CoralTarget.L3_LEFT;
    }

    @Logged(name = "CT_L3_RIGHT", importance = Importance.CRITICAL)
    public boolean coralTargetL3_RIGHT(){
        return coralTarget == CoralTarget.L3_RIGHT;
    }
    @Logged(name = "CT_L4_LEFT", importance = Importance.CRITICAL)
    public boolean coralTargetL4_LEFT(){
        return coralTarget == CoralTarget.L4_LEFT;
    }

    @Logged(name = "CT_L4_RIGHT", importance = Importance.CRITICAL)
    public boolean coralTargetL4_RIGHT(){
        return coralTarget == CoralTarget.L4_RIGHT;
    }

    public double getCoralHeight(){
        if(coralTarget == CoralTarget.L4_RIGHT || coralTarget == CoralTarget.L4_LEFT){
            return ElevatorConstants.FLOOR.LEVEL_4.position;
        }
        else if(coralTarget == CoralTarget.L3_RIGHT || coralTarget == CoralTarget.L3_LEFT){
            return ElevatorConstants.FLOOR.LEVEL_3.position;
        }
        else if(coralTarget == CoralTarget.L2_RIGHT || coralTarget == CoralTarget.L2_LEFT){
            return ElevatorConstants.FLOOR.LEVEL_2.position;
        }
        else if(coralTarget == CoralTarget.TROUGH){
            return ElevatorConstants.FLOOR.TROUGH.position;
        }
        else{
            return 0;
        }
    }

    public void setHasCoral(boolean bool){
        hasCoral = bool;
    }

    public boolean hasCoral(){
        return hasCoral;
    }

    public PhotonCamera pickReefCamera(){
        //LEFT CAMERA IS ZERO
        //ALIGN TO LEFT POST IS LEFT CAMERA (I HOPE!)
        CoralState c = CoralState.getInstance();
        if (c.coralTargetL2_LEFT() || c.coralTargetL3_LEFT() || c.coralTargetL4_LEFT() || c.coralTarget_TROUGH()){
          return CameraConstants.photonCameraOuttakeLeft;
        }
        else{
          return CameraConstants.photonCameraOuttakeRight;
        }
      }

    public Transform3d getCameraTransform(){
        //LEFT CAMERA IS ZERO
        //ALIGN TO LEFT POST IS LEFT CAMERA (I HOPE!)
        CoralState c = CoralState.getInstance();
        if (c.coralTargetL2_LEFT() || c.coralTargetL3_LEFT() || c.coralTargetL4_LEFT() || c.coralTarget_TROUGH()){
          return CameraConstants.photonCameraTransformOuttakeLeft;
        }
        else{
          return CameraConstants.photonCameraTransformOuttakeRight;
        }
      }

    public double getYCoralOffsetMeters(){
        if(coralTarget == CoralTarget.L4_LEFT || coralTarget == CoralTarget.L3_LEFT || coralTarget == CoralTarget.L2_LEFT){
            return Preferences.alignYOffsetLeft.getValue();
        }
        else if(coralTarget == CoralTarget.L4_RIGHT || coralTarget == CoralTarget.L3_RIGHT || coralTarget == CoralTarget.L2_RIGHT){
            return Preferences.alignYOffsetRight.getValue();
        }
        else return 0.0;
    }
}
