package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

public class CameraConstants {
    public static final String photonCameraNameFrontLeft = "FRONT_LEFT";
    public static final Transform3d photonCameraTransformFrontLeft = new Transform3d(new Translation3d(.343, 0.271, .239), new Rotation3d(0.0, -10 / 180.0 * Math.PI, -15.0/180 * Math.PI));
    
    public static final String photonCameraNameFrontRight = "FRONT_RIGHT";
    public static final Transform3d photonCameraTransformFrontRight = new Transform3d(new Translation3d(.343, -0.271, .239), new Rotation3d(0.0, -10 / 180.0 * Math.PI, 15.0/180 * Math.PI));
    
    public static final Integer[] IGNORED_POSE_TARGETS = {50,51};
}
