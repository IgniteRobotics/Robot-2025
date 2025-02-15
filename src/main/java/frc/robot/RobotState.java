package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;

import java.util.HashMap;
import java.util.Map;

import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.generated.TunerConstants;
import frc.zones.Grid;
import frc.zones.Zone;

@Logged
public class RobotState {

    private static RobotState single_instance = null;

    private static Pose2d robotPose2d;

    private final double blockWidth = 0.25;

    private Grid fieldGrid = new Grid();

    private Zone currentZone;

    Map<String, PhotonPipelineResult> cameraResults = new HashMap<>(){};

    private RobotState() {
    }

    public static synchronized RobotState getInstance()
    {
        if (single_instance == null)
            single_instance = new RobotState();
        
        return single_instance;
    }

    private void updateZone(){
        if(robotPose2d == null 
        || (int)(robotPose2d.getX()/blockWidth) >= fieldGrid.xLength || (int)(robotPose2d.getY()/blockWidth) >= fieldGrid.yLength
        || robotPose2d.getX() < 0 || robotPose2d.getY() < 0){
            currentZone = null;
        }
        else{
            currentZone = fieldGrid.GRID[(int)(robotPose2d.getX()/blockWidth)][ (int)(robotPose2d.getY()/blockWidth)];
        }
    }

    public Zone getZone(){
        return currentZone;
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
            return TunerConstants.DrivetrainConstants.kSpeedAt12Volts.in(MetersPerSecond);
        }
        else return getZone().maxSpeed.doubleValue();
    }

    @Logged(name = "Max Rotation", importance = Importance.CRITICAL)
    public double getMaxRotation(){
        if(getZone() == null){
            return TunerConstants.DrivetrainConstants.MAX_ANGULAR_SPEED;
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
}
