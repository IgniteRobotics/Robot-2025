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
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
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

        private Transform3d transform3d;

        private String cameraName;

        private int m_tag_Id;

        public TargetInfo(Transform3d transform, double yaw, int tag_Id, String name){
            this.transform3d = transform;
            this.yaw = yaw;
            cameraName = name;
            m_tag_Id = tag_Id;
        }

        public double getYaw() {
            return yaw;
        }

        public void setYaw(double yaw) {
            this.yaw = yaw;
        }

        public Transform3d getTransform(){
            return transform3d;
        }

        public void setTransform(Transform3d transform){
            transform3d = transform;
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

    public PhotonPoseEstimator photonPoseEstimatorOuttakeLeft;
    public PhotonPoseEstimator photonPoseEstimatorOuttakeRight;
    public PhotonPoseEstimator photonPoseEstimatorIntake;

    public PhotonPoseEstimator allEstimators[] = new PhotonPoseEstimator[3];
    

    public AprilTagFieldLayout layout;

    public static enum Side {
        OUTTAKE_LEFT, OUTTAKE_RIGHT, INTAKE
    }

    public PhotonCameraWrapper() {
        
        try {
            layout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);

        } catch (UncheckedIOException e) {
            e.printStackTrace();
        }

        //TODO: investigate PNP on the co-proc.
        photonPoseEstimatorOuttakeLeft = new PhotonPoseEstimator(layout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, CameraConstants.photonCameraTransformOuttakeLeft);
        photonPoseEstimatorOuttakeRight = new PhotonPoseEstimator(layout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, CameraConstants.photonCameraTransformOuttakeRight);
        photonPoseEstimatorIntake = new PhotonPoseEstimator(layout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
        CameraConstants.photonCameraTransformIntake);
       
        allEstimators[0] = photonPoseEstimatorOuttakeLeft;
        allEstimators[1] = photonPoseEstimatorOuttakeRight;
        allEstimators[2] = photonPoseEstimatorIntake;

    }

    public ArrayList<Optional<EstimatedRobotPose>> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose, Side side) {
        if(side == Side.OUTTAKE_RIGHT) {
            photonPoseEstimatorOuttakeRight.setReferencePose(prevEstimatedRobotPose);
            var results = CameraConstants.photonCameraOuttakeRight.getAllUnreadResults();


            if(!results.isEmpty()){
                var latestResult = results.get(results.size()-1);
                m_driveState.setLatestPhotonVisionResult(CameraConstants.photonCameraOuttakeRight.getName(), latestResult);
            }

            ArrayList<Optional<EstimatedRobotPose>> estimatedPoses = new ArrayList<Optional<EstimatedRobotPose>>();
            for(var result : results){
                 estimatedPoses.add(photonPoseEstimatorOuttakeRight.update(result));
            }
            return estimatedPoses;


        } else if(side == Side.OUTTAKE_LEFT){
            photonPoseEstimatorOuttakeLeft.setReferencePose(prevEstimatedRobotPose);
            var results = CameraConstants.photonCameraOuttakeLeft.getAllUnreadResults();
            
            if(!results.isEmpty()){
                var latestResult = results.get(results.size()-1);
                m_driveState.setLatestPhotonVisionResult(CameraConstants.photonCameraOuttakeLeft.getName(), latestResult);
            }

            ArrayList<Optional<EstimatedRobotPose>> estimatedPoses = new ArrayList<Optional<EstimatedRobotPose>>();
            for(var result : results){
                    estimatedPoses.add(photonPoseEstimatorOuttakeLeft.update(result));

            }
            return estimatedPoses;
        }
        else{
            photonPoseEstimatorIntake.setReferencePose(prevEstimatedRobotPose);
            var results = CameraConstants.photonCameraIntake.getAllUnreadResults();
            
            if(!results.isEmpty()){
                var latestResult = results.get(results.size()-1);
                m_driveState.setLatestPhotonVisionResult(CameraConstants.photonCameraIntake.getName(), latestResult);
            }

            ArrayList<Optional<EstimatedRobotPose>> estimatedPoses = new ArrayList<Optional<EstimatedRobotPose>>();
            for(var result : results){
                    estimatedPoses.add(photonPoseEstimatorIntake.update(result));
            }
            return estimatedPoses;
        }
    }

    public ArrayList<Optional<EstimatedRobotPose>> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
        return getEstimatedGlobalPose(prevEstimatedRobotPose, Side.INTAKE);
    }


    public Optional<TargetInfo> seekOuttakeTargets(int[] ids, int cameraId){

        //0 is left, 1 is right
        PhotonCamera cam = CameraConstants.outtakeCameras[cameraId];

        ArrayList< Optional<PhotonTrackedTarget> > targets = new ArrayList< Optional<PhotonTrackedTarget> >();

        var newResult = m_driveState.getLatestPhotonVisionResult(cam.getName());
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

            return Optional.of(new TargetInfo(target.get().getBestCameraToTarget(),
                target.get().getYaw(), target.get().getFiducialId(), cam.getName()));
        }
        
        
        return Optional.empty();

    }

    public Optional<TargetInfo> seekIntakeTargets(int[] ids){

        PhotonCamera cam = CameraConstants.photonCameraIntake;

        ArrayList< Optional<PhotonTrackedTarget> > targets = new ArrayList< Optional<PhotonTrackedTarget> >();

        var newResult = m_driveState.getLatestPhotonVisionResult(cam.getName());
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

            return Optional.of(new TargetInfo(target.get().getBestCameraToTarget(),
                target.get().getYaw(), target.get().getFiducialId(), cam.getName()));
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

    public void setPipeline(int index){
        CameraConstants.photonCameraOuttakeLeft.setPipelineIndex(index);
        CameraConstants.photonCameraOuttakeRight.setPipelineIndex(index);
    }

    public static boolean contains(final int[] arr, final int key) {
        return Arrays.stream(arr).anyMatch(i -> i == key);
    }

}
