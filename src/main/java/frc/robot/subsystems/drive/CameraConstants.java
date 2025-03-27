package frc.robot.subsystems.drive;

import java.util.HashMap;
import java.util.function.DoubleSupplier;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

public class CameraConstants {

    private static final AprilTagFieldLayout layout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);

    public static final String photonCameraNameOuttakeLeft = "OUTTAKE_LEFT";
    public static final Transform3d photonCameraTransformOuttakeLeft= new Transform3d(new Translation3d(-.1277, 0.2667, .4964), new Rotation3d(0.0, -15 / 180.0 * Math.PI, Math.PI));
    public static final PhotonCamera photonCameraOuttakeLeft = new PhotonCamera(photonCameraNameOuttakeLeft);
    public static final PhotonPoseEstimator photonPoseEstimatorOuttakeLeft = new PhotonPoseEstimator(layout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, photonCameraTransformOuttakeLeft);

    public static final String photonCameraNameOuttakeRight = "OUTTAKE_RIGHT";
    public static final Transform3d photonCameraTransformOuttakeRight = new Transform3d(new Translation3d(-.1277, - 0.2667, .4964), new Rotation3d(0.0, -15 / 180.0 * Math.PI, Math.PI));
    public static final PhotonCamera photonCameraOuttakeRight = new PhotonCamera(photonCameraNameOuttakeRight);
    public static final PhotonPoseEstimator photonPoseEstimatorOuttakeRight = new PhotonPoseEstimator(layout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, photonCameraTransformOuttakeRight);

    public static final String photonCameraNameIntake = "INTAKE";
    public static final Transform3d photonCameraTransformIntake = new Transform3d(new Translation3d(.343, -0.271, .239), new Rotation3d(0.0, 15 / 180.0 * Math.PI,  0));
    public static final PhotonCamera photonCameraIntake = new PhotonCamera(photonCameraNameIntake);
    public static final PhotonPoseEstimator photonPoseEstimatorIntake = new PhotonPoseEstimator(layout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, CameraConstants.photonCameraTransformIntake);


    public static final PhotonPoseEstimator[] allPhotonPoseEstimators= {photonPoseEstimatorOuttakeLeft, photonPoseEstimatorOuttakeRight, photonPoseEstimatorIntake};
    
    public static final Integer[] IGNORED_POSE_TARGETS = {50,51};
    public static final HashMap<String, Double> offsetToBumper = new HashMap<String, Double>(){{
        put("OUTTAKE_LEFT", 0.3429);
        put("OUTTAKE_RIGHT", 0.3429);
    }};

    public static final double yLeftError = 0.5;
    public static final double yRightError = -0.5;

    //sets of tag ids for targeting
    public static final int[] RED_HUMAN_PLAYER_TAGS = {1,2};
    public static final int[] BLUE_HUMAN_PLAYER_TAGS = {12,13};
    public static final int[] RED_REEF_TAGS = {6,7,8,9,10,11};
    public static final int[] BLUE_REEF_TAGS = {17,18,19,20,21,22};
    public static final int[] RED_PROCESSOR_TAGS = {3};
    public static final int[] BLUE_PROCESSOR_TAGS = {16};
    public static final int[] RED_BARGE_TAGS = {5,15};
    public static final int[] BLUE_BARGE_TAGS = {4,14};

    public static final double CORAL_REEF_STOP_DISTANCE_METERS = 0.359;
    
    public static final double ALGAE_REEF_STOP_DISTANCE_METERS = 1.0;
    
    public static final double ALGAE_PROCESSOR_STOP_DISTANCE_METERS = 0.5;

    public static final double MINIMUM_AMBIGUITY = 0.7;


    //this is the offset, in meters from the center of the tag,
    //to the center of the camera
    //when the robot is centered on the tag at the given BUMPER! distance
    public static final double getXOffsetMeters(double distance) {
        return distance + offsetToBumper.get(photonCameraNameOuttakeLeft);
    }
    
    //this is the offset, in meters from the center of the tag,
    //to the center of the camera
    //when the robot is centered on the tag at the given BUMPER! distance
    public static final double getAlgaeYawOffsetDegreesLeft(double distance) {
        
        return Math.toDegrees(photonCameraTransformOuttakeLeft.getY()
            /(distance + offsetToBumper.get(photonCameraNameOuttakeLeft)));
    }

    public static final double getAlgaeYawOffsetDegreesRight(double distance) {
        return Math.toDegrees(photonCameraTransformOuttakeRight.getY()
        /(distance + offsetToBumper.get(photonCameraNameOuttakeRight)));
    }

    //this is the offset, in meters from the center of the tag,
    //to the center of the camera
    //when the robot is centered on the reef branch
    //at the given BUMPER! distance
    public static final double getCorallYawOffsetDegreesLeft(double distance) {
        double yOffset = -0.132;
        return Math.toDegrees(Math.asin(yOffset/(distance)));
    }
    //this is the offset, in meters from the center of the tag,
    //to the center of the camera=        
    //when the robot is centered on the reef branch
    public static final double getCorallYawOffsetDegreesRight(double distance) {
        double yOffset = 0.132;
        return Math.toDegrees(Math.asin(yOffset/(distance)));
    }

}

