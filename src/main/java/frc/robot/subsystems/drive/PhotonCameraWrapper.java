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
import frc.robot.RobotState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class PhotonCameraWrapper{

    private boolean m_seesTarget;
    
    private double m_yawRadians;
    private double m_currentCameraOffset;

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
    private PhotonCamera  m_targetCam = null;
    private PhotonPoseEstimator m_targetEstimator = null;
    private Timer m_targetTimer = new Timer();
    private double m_lockTimeSec = 0.2;

    public static enum Side {
        OUTTAKE_LEFT, OUTTAKE_RIGHT, INTAKE
    }

    public PhotonCameraWrapper() {
        
        try {
            //TODO: Change as soon as possible
            layout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
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
        
        //we're using a camera that was already locked on
        if (null != m_targetCam){  
            var result = m_robotState.getLatestPhotonVisionResult(m_targetCam.getName());
            if(result != null){
                Optional<PhotonTrackedTarget> target = lookForTarget(result, id);
                if (target.isPresent()){
                    m_targetTimer.restart();
                return Optional.of(calculateTargetInfo(
                        target.get().getYaw(), 
                        getDistanceFromTransform3d(target.get().getBestCameraToTarget()),
                        m_currentCameraOffset,
                        m_targetEstimator.getRobotToCameraTransform().getY(),
                        m_targetCam.getName()
                        ));
                }
            }

            //we didn't find it with the locked camera.
            //if the timer has expired, go back to all cams.
            if(m_targetTimer.hasElapsed(m_lockTimeSec)){
                m_seesTarget = false;
                m_yawRadians = 0;
                m_targetCam = null;
                m_targetEstimator = null;
                m_targetTimer.stop();
            }
        } else { //no pre locked camera.  loop through them all.
            PhotonCamera designatedCameras[] = CameraConstants.targetCameras.get(id);
            for (int i = 0; i < designatedCameras.length; i++){
                var newResult = m_robotState.getLatestPhotonVisionResult(designatedCameras[i].getName());
                if(newResult != null){
                    Optional<PhotonTrackedTarget> target = lookForTarget(newResult, id);
                    if (target.isPresent()){
                        m_seesTarget = true;
                        m_targetCam = designatedCameras[i];
                        m_targetEstimator = allEstimators[i];
                        m_currentCameraOffset = CameraConstants.allCameraYawOffsetsDegrees[i];
                        m_targetTimer.restart();
                        //return Optional.of(new TargetInfo(getDistanceFromTransform3d(target.get().getBestCameraToTarget()), target.get().getYaw()));
                        //return Optional.of(buildTargetInfo(target.get().getBestCameraToTarget(), m_targetEstimator.getRobotToCameraTransform()));
                        return Optional.of(calculateTargetInfo(
                            target.get().getYaw(), 
                            getDistanceFromTransform3d(target.get().getBestCameraToTarget()),
                            m_currentCameraOffset,
                            m_targetEstimator.getRobotToCameraTransform().getY(),
                            m_targetCam.getName()
                        ));
                    }
                }
            }
        }
        
        //no targets found anywhere.
        m_targetCam = null;
        m_targetEstimator = null;
        m_targetTimer.stop();
        m_seesTarget = false;
        m_currentCameraOffset = 0;
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
            return Optional.of(new PhotonTrackedTarget(0.2, 0.0, 1.0, 0.0, targetId, -1, -1,
                new Transform3d(1, 1, 1, new Rotation3d(0.0, 0.0, 0.2)),
                new Transform3d(1, 1, 1, new Rotation3d(0.0, 0.0, 0.2)),
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


    public void unlockTargeting(){
        m_targetCam = null;
        m_targetEstimator = null;
        m_yawRadians = 0.0;
        m_targetTimer.stop();
        m_currentCameraOffset = 0;
    }

    public TargetInfo calculateTargetInfo(double yawToTargetDegrees, double distanceToTargetMeters, double cameraYawOffset, double cameraYOffsetMeters, String cameraName){
        //first offset the yaw by the camera angle and 90.
        yawToTargetDegrees = yawToTargetDegrees + cameraYawOffset + 90;

        //apply law of cosines to get robot distance
        double distance = Math.sqrt(
            Math.pow(cameraYOffsetMeters, 2) +
            Math.pow(distanceToTargetMeters, 2) -
            (
                2 * cameraYOffsetMeters * distanceToTargetMeters *
                Math.cos(Math.toRadians(yawToTargetDegrees))
            )
            );

        //now apply law of sines to get robot yaw
        // and flip to degrees.
        double yaw = Math.toDegrees(Math.asin(
            distanceToTargetMeters * Math.sin(Math.toRadians(yawToTargetDegrees)) /
            distance)
        );

        //finally, subract 90 deg from yaw to get yaw from straigh ahead.
        yaw -= 90;

        //round to 2 places.
        distance = Math.round(distance*100.0)/100.0;
        yaw = Math.round(yaw * 100.0)/100.0;

        TargetInfo t = new TargetInfo(distance, yaw, cameraName);

        return t;
        
    }

    public void setPipeline(int index){
        CameraConstants.photonCameraOuttakeLeft.setPipelineIndex(index);
        CameraConstants.photonCameraOuttakeRight.setPipelineIndex(index);
    }

}
