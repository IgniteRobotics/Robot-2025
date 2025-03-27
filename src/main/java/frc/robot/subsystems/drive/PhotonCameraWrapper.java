package frc.robot.subsystems.drive;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import java.io.UncheckedIOException;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.cameraserver.CameraServerSharedStore;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N3;
import frc.robot.Robot;
import frc.robot.statemachines.DriveState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class PhotonCameraWrapper{

    private boolean m_seesTarget;
    
    private double m_yawRadians;

    DriveState m_driveState = DriveState.getInstance();

    public class TargetInfo{
        private double yaw;

        private double distance;

        private String cameraName;

        private int m_tag_Id;

        private Transform3d m_transform3d;

        public TargetInfo(double distance, Transform3d transform3d, double yaw, int tag_Id, String name){
            this.distance = distance;
            this.yaw = yaw;
            cameraName = name;
            m_transform3d = transform3d;
            m_tag_Id = tag_Id;
        }

        public double getYaw() {
            return yaw;
        }

        public Transform3d getTransform3d(){
            return m_transform3d;
        }

        public void setYaw(double yaw) {
            this.yaw = yaw;
        }

        public double getDistance() {
            return distance;
        }

        public void setDistance(double distance) {
            this.distance = distance;
        }

        public int getTagId(){
            return this.m_tag_Id;
        }

        public void setTagId(int tag_Id){
            this.m_tag_Id = tag_Id;
        }

        public String getCameraName(){
            return cameraName;
        }
    }

    public PhotonCameraWrapper() {
        

        //TODO: investigate PNP on the co-proc.
        
    }

    public Optional<TargetInfo> seekTargets(int[] ids, PhotonCamera camera){

        ArrayList< Optional<PhotonTrackedTarget> > targets = new ArrayList< Optional<PhotonTrackedTarget> >();

        var newResult = m_driveState.getLatestPhotonVisionResult(camera.getName());
        if(newResult != null){
            for (int id : ids) {
                Optional<PhotonTrackedTarget> tempTarget = lookForTarget(newResult, id);
                if(tempTarget.isPresent() && tempTarget.get().getPoseAmbiguity() < CameraConstants.MINIMUM_AMBIGUITY){
                    targets.add(tempTarget);
                }
            }
        }

        if(targets.size() > 0){
            Optional<PhotonTrackedTarget> target = targets.get(0);
            double maxArea = target.get().getArea();
            for(int i = 1; i < targets.size(); i++){
                if(maxArea < targets.get(i).get().getArea()){
                    target = targets.get(i);
                    maxArea = target.get().getArea();
                }
            }

            return Optional.of(new TargetInfo((getDistanceFromTransform3d(target.get().getBestCameraToTarget())), target.get().getBestCameraToTarget(),
                target.get().getYaw(), target.get().getFiducialId(), camera.getName()));
        }
        
        
        return Optional.empty();

    }

    private Optional<PhotonTrackedTarget> lookForTarget(PhotonPipelineResult result, int targetId){
        for (var target : result.getTargets()){
                if (targetId == target.getFiducialId()){
                    return Optional.of(target) ;
                }
            }

        if (Robot.isReal()){
            return Optional.empty();
        } else {
            //TODO: Change values (after TargetId 2 addition values, see class)
            return Optional.of(new PhotonTrackedTarget(0, 0.0, 0, 0.0, targetId, -1, -1,
                new Transform3d(1, 1, 1, new Rotation3d(0.0, 0.0, 0)),
                new Transform3d(1, 1, 1, new Rotation3d(0.0, 0.0, 0)),
             0.0, 
             new ArrayList<TargetCorner>(4), 
             new ArrayList<TargetCorner>(4)
             ));
        }
    }


    private double getDistanceFromTransform3d(Transform3d t){
        return Math.sqrt(
                Math.pow(t.getX(), 2) + 
                Math.pow(t.getY(), 2)
        );
    }

    public void setPipeline(int index){
        CameraConstants.photonCameraOuttakeLeft.setPipelineIndex(index);
        CameraConstants.photonCameraOuttakeRight.setPipelineIndex(index);
    }

    public static boolean contains(final int[] arr, final int key) {
        return Arrays.stream(arr).anyMatch(i -> i == key);
    }

}
