package frc.robot.subsystems.drive;
import java.util.ArrayList;
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
import frc.robot.statemachines.RobotState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class PhotonCameraWrapper{

    private boolean m_seesTarget;
    
    private double m_yawRadians;

    RobotState m_robotState = RobotState.getInstance();

    public class TargetInfo{
        private double yaw;

        private double distance;

        private String cameraName;

        public TargetInfo(double distance, double yaw, String name){
            this.distance = distance;
            this.yaw = yaw;
            cameraName = name;
        }

        public double getYaw() {
            return yaw;
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
                m_robotState.setLatestPhotonVisionResult(CameraConstants.photonCameraOuttakeRight.getName(), latestResult);
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
                m_robotState.setLatestPhotonVisionResult(CameraConstants.photonCameraOuttakeLeft.getName(), latestResult);
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
                m_robotState.setLatestPhotonVisionResult(CameraConstants.photonCameraIntake.getName(), latestResult);
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

    public Optional<TargetInfo> seekTarget(int id){

        //loop through all cameras to find the one with least ambiguity
        PhotonCamera designatedCameras[] = CameraConstants.targetCameras.get(id);
        double minimumAmbiguity = 1;
        int bestCamera = -1;
        Optional<PhotonTrackedTarget> target = Optional.empty();
        for (int i = 0; i < designatedCameras.length; i++){
            var newResult = m_robotState.getLatestPhotonVisionResult(designatedCameras[i].getName());
            if(newResult != null){
                Optional<PhotonTrackedTarget> tempTarget = lookForTarget(newResult, id);
                if(tempTarget.isPresent() && tempTarget.get().getPoseAmbiguity() < minimumAmbiguity){
                    bestCamera = i;
                    minimumAmbiguity = tempTarget.get().getPoseAmbiguity();
                    target = tempTarget;
                }
            }
        }

        //the best camera, if any, is used
        if(bestCamera != -1){
            m_seesTarget = true;
            return Optional.of(new TargetInfo((getDistanceFromTransform3d(target.get().getBestCameraToTarget()) - CameraConstants.offsetToBumper.get(designatedCameras[bestCamera].getName())),
                target.get().getYaw(), designatedCameras[bestCamera].getName()));
        }
        
        //no targets found anywhere.
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

}
