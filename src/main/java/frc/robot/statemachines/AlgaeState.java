// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.statemachines;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import frc.robot.subsystems.Elevator.ElevatorConstants;
import frc.robot.subsystems.algae.AlgaeCollectorConstants;
import frc.zones.ZoneTypes;

@Logged
public class AlgaeState {
    private static AlgaeState single_instance = null;

    private DriveState driveState = DriveState.getInstance();

    private AlgaeState() {

    }

    public static synchronized AlgaeState getInstance()
    {
        if (single_instance == null)
            single_instance = new AlgaeState();
        
        return single_instance;
    }

    private boolean hasAlgae = false;

    //Desired Algae Targets
    public static enum AlgaeTarget {
        NONE("NONE"),
        PROCESSOR("PROCESSOR"),
        REEF("REEF"),
        BARGE("BARGE");

        public final String name;
        AlgaeTarget(String value){
            name = value;
        }
    }

    private AlgaeTarget algaeTarget = AlgaeTarget.NONE;

    public void setAlgaeTarget(AlgaeTarget target){
        algaeTarget = target;
    }

    public AlgaeTarget getAlgaeTarget(){
        return algaeTarget;
    }

    @Logged(name = "Algae Target", importance = Importance.CRITICAL)
    public String getAlgaeTargetName(){
        return algaeTarget.name;
    }

    @Logged(name = "AT_PROCESSOR", importance = Importance.CRITICAL)
    public boolean algaeTargetPROCESSOR(){
        return algaeTarget == AlgaeTarget.PROCESSOR;
    }

    @Logged(name = "AT_REEF", importance = Importance.CRITICAL)
    public boolean algaeTargetREEF_L2(){
        return algaeTarget == AlgaeTarget.REEF;
    }


    @Logged(name = "AT_BARGE", importance = Importance.CRITICAL)
    public boolean algaeTargetBARGE(){
        return algaeTarget == AlgaeTarget.BARGE;
    }

    public double getAlgaeHeight(){
        if(getAlgaeTarget() == AlgaeTarget.BARGE){
            return ElevatorConstants.ALGAE.BARGE.height;
        }
        else if (getAlgaeTarget() == AlgaeTarget.PROCESSOR){
            return ElevatorConstants.ALGAE.PROCESSOR.height;
        }
        else if (getAlgaeTarget() == AlgaeTarget.REEF){
            if(driveState.getZone() == ZoneTypes.REEF.REEF_AB || driveState.getZone() == ZoneTypes.REEF.REEF_EF || driveState.getZone() == ZoneTypes.REEF.REEF_IJ){
                return ElevatorConstants.ALGAE.HIGH_REEF.height;
            }
            else{ 
                return ElevatorConstants.ALGAE.LOW_REEF.height;
            }
        } else {
            return 0;
        }
    }

    public double getAlgaeWristPosition(){
        if(getAlgaeTarget() == AlgaeTarget.BARGE){
            return AlgaeCollectorConstants.ALGAE.BARGE.angle;
        }
        else if (getAlgaeTarget() == AlgaeTarget.PROCESSOR){
            return AlgaeCollectorConstants.ALGAE.PROCESS.angle;
        }
        else if (getAlgaeTarget() == AlgaeTarget.REEF){
            if(driveState.getZone() == ZoneTypes.REEF.REEF_AB || driveState.getZone() == ZoneTypes.REEF.REEF_EF || driveState.getZone() == ZoneTypes.REEF.REEF_IJ){
                return AlgaeCollectorConstants.ALGAE.REEF.angle;
            }
            else{ 
                return AlgaeCollectorConstants.ALGAE.REEF.angle;
            }
        } else {
            if (hasAlgae) {
                return AlgaeCollectorConstants.ALGAE.STOW_FULL.angle;
            } else {
                return AlgaeCollectorConstants.ALGAE.STOW_EMPTY.angle;
            }
        }
    }

    public void setHasAlgae(boolean bool){
        hasAlgae = bool;
    }

    public boolean hasAlgae(){
        return hasAlgae;
    }

}
