package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;

import java.util.HashMap;
import java.util.Map;
import java.util.function.Supplier;

import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Elevator.ElevatorConstants;
import frc.robot.subsystems.EndEffector.EndEffectorConstants;
import frc.robot.subsystems.drive.CameraConstants;
import frc.zones.Grid;
import frc.zones.Zone;
import frc.zones.ZoneTypes;

@Logged
public class RobotState {

    private static RobotState single_instance = null;

    private static Pose2d robotPose2d;

    private final double blockWidth = 0.25;

    private Zone currentZone;

    Map<String, PhotonPipelineResult> cameraResults = new HashMap<>(){};

    private boolean hasCoral = false;

    private boolean hasAlgae = false;

    private Alliance m_alliance = Alliance.Red;


    public void setAlliance(Alliance alliance){
        m_alliance = alliance;
    }

    public Alliance getAlliance(){
        return m_alliance;
    }

    public double getBargeHeading(){
        if(m_alliance == Alliance.Red){
            return 90;
        }
        else return 0;
    }
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

    @Logged(name = "Coral Target", importance = Importance.CRITICAL)
    public String getCoralTargetName(){
        return coralTarget.name;
    }

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

    @Logged(name = "Algae Target", importance = Importance.CRITICAL)
    public String getAlgaeTargetName(){
        return algaeTarget.name;
    }

    private RobotState() {

    }

    public static synchronized RobotState getInstance()
    {
        if (single_instance == null)
            single_instance = new RobotState();
        
        return single_instance;
    }

    public Zone[][] getGrid(){
        if(m_alliance == Alliance.Red){
            return Grid.RED_GRID;
        }
        else return Grid.BLUE_GRID;
    }

    
    private void updateZone(){
        if(robotPose2d == null 
        || (int)(robotPose2d.getX()/blockWidth) >= Grid.xLength || (int)(robotPose2d.getY()/blockWidth) >= Grid.yLength
        || robotPose2d.getX() < 0 || robotPose2d.getY() < 0){
            currentZone = null;
        }
        else{
            currentZone = getGrid()[(int)(robotPose2d.getX()/blockWidth)][ (int)(robotPose2d.getY()/blockWidth)];
        }
    }

    @NotLogged
    public Zone getZone(){
        return currentZone;
    }

    @Logged(name = "Zone", importance = Importance.CRITICAL)
    public String getZoneName(){
        if(currentZone == null) return "currentZone is nonexistent";
        else return currentZone.name;
    }

    public double getAlgaeHeight(){
        if(getAlgaeTarget() == AlgaeTarget.BARGE){
            return ElevatorConstants.ALGAE.BARGE.height;
        }
        else if (getAlgaeTarget() == AlgaeTarget.PROCESSOR){
            return ElevatorConstants.ALGAE.PROCESSOR.height;
        }
        else if (getAlgaeTarget() == AlgaeTarget.REEF){
            if(getZone() == ZoneTypes.REEF.REEF_AB || getZone() == ZoneTypes.REEF.REEF_EF || getZone() == ZoneTypes.REEF.REEF_IJ){
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
            return EndEffectorConstants.ALGAE.BARGE.angle;
        }
        else if (getAlgaeTarget() == AlgaeTarget.PROCESSOR){
            return EndEffectorConstants.ALGAE.PROCESS.angle;
        }
        else if (getAlgaeTarget() == AlgaeTarget.REEF){
            if(getZone() == ZoneTypes.REEF.REEF_AB || getZone() == ZoneTypes.REEF.REEF_EF || getZone() == ZoneTypes.REEF.REEF_IJ){
                return EndEffectorConstants.ALGAE.REEF.angle;
            }
            else{ 
                return EndEffectorConstants.ALGAE.REEF.angle;
            }
        } else {
            if (hasAlgae) {
                return EndEffectorConstants.ALGAE.STOW_FULL.angle;
            } else {
                return EndEffectorConstants.ALGAE.STOW_EMPTY.angle;
            }
        }
    }

    public synchronized void setPose2d(Pose2d newPose){
        robotPose2d = newPose;
        updateZone();
    }

    public Pose2d getPose2d(){
        return robotPose2d;
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

    public void setLatestPhotonVisionResult(String camera, PhotonPipelineResult newResult){
        cameraResults.put(camera, newResult);
    }

    public PhotonPipelineResult getLatestPhotonVisionResult(String camera){
        if(cameraResults.containsKey(camera))return cameraResults.get(camera);
        else return null;
    }

  

    public void setCoralTarget(CoralTarget target){
        coralTarget = target;
    }
    
    public CoralTarget getCoralTarget(){
        return coralTarget;
    }

    @Logged(name = "CT_TROUGH", importance = Importance.CRITICAL)
    public boolean coralTargetTrough(){
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

    public double getYAlignmentError(){
        if(coralTarget == CoralTarget.L4_LEFT || coralTarget == CoralTarget.L3_LEFT || coralTarget == CoralTarget.L2_LEFT){
            return CameraConstants.yLeftError;
        }
        else if(coralTarget == CoralTarget.L4_RIGHT || coralTarget == CoralTarget.L3_RIGHT || coralTarget == CoralTarget.L2_RIGHT){
            return CameraConstants.yRightError;
        }
        else return 0;
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

    public void setAlgaeTarget(AlgaeTarget target){
        algaeTarget = target;
    }

    public AlgaeTarget getAlgaeTarget(){
        return algaeTarget;
    }

    public void setHasCoral(boolean bool){
        hasCoral = bool;
    }

    public boolean hasCoral(){
        return hasCoral;
    }

    public void setHasAlgae(boolean bool){
        hasAlgae = bool;
    }

    public boolean hasAlgae(){
        return hasAlgae;
    }
}

