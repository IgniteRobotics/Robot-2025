package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.generated.TunerConstants;
import frc.zones.Grid;
import frc.zones.Zone;

public class RobotState {

    private static RobotState single_instance = null;

    private static Pose2d robotPose2d;

    private double blockWidth = 0.3;

    private Grid fieldGrid = new Grid();

    private RobotState() {
    }

    public static synchronized RobotState getInstance()
    {
        if (single_instance == null)
            single_instance = new RobotState();
        
        return single_instance;
    }

    public synchronized void setPose2d(Pose2d newPose){
        robotPose2d = newPose;
    }

    public Pose2d getPose2d(){
        return robotPose2d;
    }

    public double getMaxSpeed(){
    
        if(robotPose2d == null 
        || (int)(robotPose2d.getX()/blockWidth) > fieldGrid.GRID.length || (int)(robotPose2d.getY()/blockWidth) > fieldGrid.GRID[0].length
        || robotPose2d.getX() < 0 || robotPose2d.getY() < 0){
            return TunerConstants.DrivetrainConstants.kSpeedAt12Volts.in(MetersPerSecond);
        }
        return fieldGrid.GRID[(int)(robotPose2d.getX()/blockWidth)][ (int)(robotPose2d.getY()/blockWidth)].maxSpeed.doubleValue();
    }
    


}
