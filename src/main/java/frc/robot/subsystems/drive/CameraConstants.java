package frc.robot.subsystems.drive;

import java.util.HashMap;
import java.util.function.DoubleSupplier;

import org.photonvision.PhotonCamera;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

public class CameraConstants {
    public static final String photonCameraNameOuttakeLeft = "OUTTAKE_LEFT";
    public static final Transform3d photonCameraTransformOuttakeLeft= new Transform3d(new Translation3d(0.031, -0.266, .4964), new Rotation3d(0.0, -15 / 180.0 * Math.PI, Math.PI));
    public static final PhotonCamera photonCameraOuttakeLeft = new PhotonCamera(photonCameraNameOuttakeLeft);
    
    public static final String photonCameraNameOuttakeRight = "OUTTAKE_RIGHT";
    public static final Transform3d photonCameraTransformOuttakeRight = new Transform3d(new Translation3d(0.031, 0.266, .4964), new Rotation3d(0.0, -15 / 180.0 * Math.PI, Math.PI));
    public static final PhotonCamera photonCameraOuttakeRight = new PhotonCamera(photonCameraNameOuttakeRight);

    public static final String photonCameraNameIntake = "INTAKE";
    public static final Transform3d photonCameraTransformIntake = new Transform3d(new Translation3d(.343, -0.271, .239), new Rotation3d(0.0, 15 / 180.0 * Math.PI,  0));
    public static final PhotonCamera photonCameraIntake = new PhotonCamera(photonCameraNameIntake);

    public static final PhotonCamera allCameras[] = {photonCameraOuttakeLeft, photonCameraOuttakeRight, photonCameraIntake};
    public static final double allCameraYawOffsetsDegrees[] = {0,0,0};
    
    public static final PhotonCamera intakeCameras[] = {photonCameraIntake};
    public static final PhotonCamera outtakeCameras[] = {photonCameraOuttakeLeft, photonCameraOuttakeRight};
    public static final HashMap<Integer, PhotonCamera[]> targetCameras = new HashMap<Integer, PhotonCamera[]>(){{
        put(1, intakeCameras);
        put(2, intakeCameras);

        put(3, outtakeCameras);
        put(4, outtakeCameras);
        put(5, outtakeCameras);
        put(6, outtakeCameras);
        put(7, outtakeCameras);
        put(8, outtakeCameras);
        put(9, outtakeCameras);
        put(10, outtakeCameras);
        put(11, outtakeCameras);

        put(12, intakeCameras);
        put(13, intakeCameras);

        put(14, outtakeCameras);
        put(15, outtakeCameras);
        put(16, outtakeCameras);
        put(17, outtakeCameras);
        put(18, outtakeCameras);
        put(19, outtakeCameras);
        put(20, outtakeCameras);
        put(21, outtakeCameras);
        put(22, outtakeCameras);
    }};


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

