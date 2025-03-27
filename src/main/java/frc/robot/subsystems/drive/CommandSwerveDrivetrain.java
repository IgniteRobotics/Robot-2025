package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.LinkedList;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.PhotonPipelineResult;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Robot;
import frc.robot.generated.TunerConstants;
import frc.robot.generated.TunerConstants.TunerSwerveDrivetrain;
import frc.robot.statemachines.DriveState;

import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;


/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements
 * Subsystem so it can easily be used in command-based projects.
 */
@Logged
public class CommandSwerveDrivetrain extends TunerSwerveDrivetrain implements Subsystem {

    private DriveState m_driveState = DriveState.getInstance();

    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
    /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
    /* Keep track if we've ever applied the operator perspective before or not */
    private boolean m_hasAppliedOperatorPerspective = false;

     /** Swerve request to apply during robot-centric path following */
     private final SwerveRequest.ApplyRobotSpeeds m_pathApplyRobotSpeeds = new SwerveRequest.ApplyRobotSpeeds();

    /* Swerve requests to apply during SysId characterization */
    private final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization = new SwerveRequest.SysIdSwerveTranslation();
    private final SwerveRequest.SysIdSwerveSteerGains m_steerCharacterization = new SwerveRequest.SysIdSwerveSteerGains();
    private final SwerveRequest.SysIdSwerveRotation m_rotationCharacterization = new SwerveRequest.SysIdSwerveRotation();

    private final Pigeon2 m_gyro = new Pigeon2(TunerConstants.kPigeonId);

    public final PhotonCameraWrapper m_photonCameraWrapper;

    private PathPlannerPath loggedPath;

    //this holds a list of the poses of the vision targets used for positioning.
    //these are FIELD RELATIVE poses, not robot relative.
    @Logged(name = "Tag Poses", importance = Importance.CRITICAL)
    private List<Pose3d> tagPosesFieldRelative;

    //this holds a list of the tags used for positioning and some metadata.
    @Logged(name = "Tags Used", importance = Importance.CRITICAL)
    private List<TrackedAprilTag> tagsUsed;




    /* SysId routine for characterizing translation. This is used to find PID gains for the drive motors. */
    private final SysIdRoutine m_sysIdRoutineTranslation = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,        // Use default ramp rate (1 V/s)
            Volts.of(4), // Reduce dynamic step voltage to 4 V to prevent brownout
            null,        // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            output -> setControl(m_translationCharacterization.withVolts(output)),
            null,
            this
        )
    );

    /* SysId routine for characterizing steer. This is used to find PID gains for the steer motors. */
    private final SysIdRoutine m_sysIdRoutineSteer = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,        // Use default ramp rate (1 V/s)
            Volts.of(7), // Use dynamic voltage of 7 V
            null,        // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdSteer_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            volts -> setControl(m_steerCharacterization.withVolts(volts)),
            null,
            this
        )
    );

    /*
     * SysId routine for characterizing rotation.
     * This is used to find PID gains for the FieldCentricFacingAngle HeadingController.
     * See the documentation of SwerveRequest.SysIdSwerveRotation for info on importing the log to SysId.
     */
    private final SysIdRoutine m_sysIdRoutineRotation = new SysIdRoutine(
        new SysIdRoutine.Config(
            /* This is in radians per second², but SysId only supports "volts per second" */
            Volts.of(Math.PI / 6).per(Second),
            /* This is in radians per second, but SysId only supports "volts" */
            Volts.of(Math.PI),
            null, // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdRotation_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            output -> {
                /* output is actually radians per second, but SysId only supports "volts" */
                setControl(m_rotationCharacterization.withRotationalRate(output.in(Volts)));
                /* also log the requested output for SysId */
                SignalLogger.writeDouble("Rotational_Rate", output.in(Volts));
            },
            null,
            this
        )
    );

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants   Drivetrain-wide constants for the swerve drive
     * @param modules               Constants for each specific module
     */
    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        PhotonCameraWrapper cameraWrapper,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, modules);
        m_photonCameraWrapper = cameraWrapper;
        if (Utils.isSimulation()) {
            startSimThread();
        }
        configureAutoBuilder();
    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     * @param backright 
     * @param backleft 
     * @param frontright 
     * @param frontleft 
     * @param backright 
     * @param backleft 
     * @param frontright 
     * @param frontleft 
     *
     * @param drivetrainConstants     Drivetrain-wide constants for the swerve drive
     * @param odometryUpdateFrequency The frequency to run the odometry loop. If
     *                                unspecified or set to 0 Hz, this is 250 Hz on
     *                                CAN FD, and 100 Hz on CAN 2.0.
     * @param modules                 Constants for each specific module
     */
    // public CommandSwerveDrivetrain(
    //     SwerveDrivetrainConstants drivetrainConstants,
    //     double odometryUpdasteFrequency,
    //     SwerveModuleConstants<?, ?, ?>... modules
    // ) {
    //     super(drivetrainConstants, odometryUpdateFrequency, modules);
    //     if (Utils.isSimulation()) {
    //         startSimThread();
    //     }
    //     configureAutoBuilder();

    // }



    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants       Drivetrain-wide constants for the swerve drive
     * @param odometryUpdateFrequency   The frequency to run the odometry loop. If
     *                                  unspecified or set to 0 Hz, this is 250 Hz on
     *                                  CAN FD, and 100 Hz on CAN 2.0.
     * @param odometryStandardDeviation The standard deviation for odometry calculation
     *                                  in the form [x, y, theta]ᵀ, with units in meters
     *                                  and radians
     * @param visionStandardDeviation   The standard deviation for vision calculation
     *                                  in the form [x, y, theta]ᵀ, with units in meters
     *                                  and radians
     * @param modules                   Constants for each specific module
     */
    //TODO: is this needed?
    // public CommandSwerveDrivetrain(
    //     SwerveDrivetrainConstants drivetrainConstants,
    //     double odometryUpdateFrequency,
    //     Matrix<N3, N1> odometryStandardDeviation,
    //     Matrix<N3, N1> visionStandardDeviation,
    //     SwerveModuleConstants<?, ?, ?>... modules
    // ) {
    //     super(drivetrainConstants, odometryUpdateFrequency, odometryStandardDeviation, visionStandardDeviation, modules);
    //     if (Utils.isSimulation()) {
    //         startSimThread();
    //     }
    // }

    private void configureAutoBuilder() {
        try {
            var config = RobotConfig.fromGUISettings();
            AutoBuilder.configure(
                () -> getState().Pose,   // Supplier of current robot pose
                this::resetPose,         // Consumer for seeding pose against auto
                () -> getState().Speeds, // Supplier of current robot speeds
                // Consumer of ChassisSpeeds and feedforwards to drive the robot
                (speeds, feedforwards) -> setControl(
                    m_pathApplyRobotSpeeds.withSpeeds(speeds)
                        .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
                        .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())
                ),
                new PPHolonomicDriveController(
                    // PID constants for translation
                    new PIDConstants(10, 0, 0),
                    // PID constants for rotation
                    new PIDConstants(7, 0, 0)
                ),
                config,
                // Assume the path needs to be flipped for Red vs Blue, this is normally the case
                () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
                this // Subsystem for requirements
            );
        } catch (Exception ex) {
            DriverStation.reportError("Failed to load PathPlanner config and configure AutoBuilder", ex.getStackTrace());
        }
    }

    public Command followPath(PathPlannerPath path){
        loggedPath = path;
        return AutoBuilder.followPath(path);
    }

/**
    /**
     * Returns a command that applies the specified control request to this swerve drivetrain.
     *
     * @param request Function returning the request to apply
     * @return Command to run
     */
    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    public Command sysIdTranslation(){
        return m_sysIdRoutineTranslation.quasistatic(Direction.kForward).withTimeout(5).andThen(m_sysIdRoutineTranslation.quasistatic(Direction.kReverse).withTimeout(5))
            .andThen(m_sysIdRoutineTranslation.dynamic(Direction.kForward).withTimeout(3)).andThen(m_sysIdRoutineTranslation.dynamic(Direction.kReverse).withTimeout(3));
    }

    public Command sysIdRotation(){
        return m_sysIdRoutineRotation.quasistatic(Direction.kForward).withTimeout(6).andThen(m_sysIdRoutineRotation.quasistatic(Direction.kReverse).withTimeout(6))
            .andThen(m_sysIdRoutineRotation.dynamic(Direction.kForward).withTimeout(4)).andThen(m_sysIdRoutineRotation.dynamic(Direction.kReverse).withTimeout(4));
    }

    public void driveRobotCentric(double x, double y, double rot){
        SwerveRequest.RobotCentric m_driveRequest = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
            .withSteerRequestType(SteerRequestType.MotionMagicExpo);
        this.setControl(m_driveRequest.withVelocityX(x).withVelocityY(y).withRotationalRate(rot));
    }

    @Override
    public void periodic() {
        tagPosesFieldRelative = new LinkedList<Pose3d>();
        tagsUsed = new LinkedList<TrackedAprilTag>();
        if (Robot.isSimulation()){
            tagsUsed.add(new TrackedAprilTag(18, 0.45, 1.23, 0.4, -15.4, 1));
        }

        //SmartDashboard.putString("Zone", m_driveState.getZoneName());

        SmartDashboard.putNumber("Robot Pose X", getPose().getX());
        SmartDashboard.putNumber("Robot Pose Y", getPose().getY());
        SmartDashboard.putNumber("Robot Rotation Degrees", getPose().getRotation().getDegrees());




        if(loggedPath == null){
            SmartDashboard.putString("Last Selected Robot Path Name", "None Selected");
        }

        else{
            SmartDashboard.putString("Last Selected Robot Path Name", loggedPath.toString());
        }

        m_driveState.setPose2d(getPose());
        m_driveState.setYaw(getYaw());
        /*
         * Periodically try to apply the operator perspective.
         * If we haven't applied the operator perspective before, then we should apply it regardless of DS state.
         * This allows us to correct the perspective in case the robot code restarts mid-match.
         * Otherwise, only check and apply the operator perspective if the DS is disabled.
         * This ensures driving behavior doesn't change until an explicit disable event occurs during testing.
         */
        if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent(allianceColor -> {
                setOperatorPerspectiveForward(
                    allianceColor == Alliance.Red
                        ? kRedAlliancePerspectiveRotation
                        : kBlueAlliancePerspectiveRotation
                );
                m_hasAppliedOperatorPerspective = true;
            });
        }

        calculateGlobalPose();

    }

    

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }

    public Pose2d getPose(){
        return this.getState().Pose;
    }

    public void zeroHeading(){
        m_gyro.reset();
    }

    /**
     * Adds a vision measurement to the Kalman Filter. This will correct the odometry pose estimate
     * while still accounting for measurement noise.
     *
     * @param visionRobotPoseMeters The pose of the robot as measured by the vision camera.
     * @param timestampSeconds The timestamp of the vision measurement in seconds.
     */
    // @Override
    // public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds) {
    //     super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds));
    // }

    @Logged
    public double getYaw(){
        return this.getState().Pose.getRotation().getDegrees();
    }


    public void calculateGlobalPose(){
        //outtake left
        CameraConstants.photonPoseEstimatorOuttakeLeft.setReferencePose(getPose());
        var outtakeLeftResults = CameraConstants.photonCameraOuttakeLeft.getAllUnreadResults();

        if(!outtakeLeftResults.isEmpty()){
            var latestResult = outtakeLeftResults.get(outtakeLeftResults.size()-1);
            m_driveState.setLatestPhotonVisionResult(CameraConstants.photonCameraOuttakeLeft, latestResult);
        }

        for(var result: outtakeLeftResults){
            Optional<EstimatedRobotPose> estimatedPose = CameraConstants.photonPoseEstimatorOuttakeLeft.update(result);
            if(!estimatedPose.isEmpty()){
                calculateVisionMeasurement(estimatedPose.get(), 0);
            }
        }

        //outtake right
        CameraConstants.photonPoseEstimatorOuttakeRight.setReferencePose(getPose());
        var outtakeRightResults = CameraConstants.photonCameraOuttakeRight.getAllUnreadResults();

        if(!outtakeRightResults.isEmpty()){
            var latestResult = outtakeRightResults.get(outtakeRightResults.size()-1);
            m_driveState.setLatestPhotonVisionResult(CameraConstants.photonCameraOuttakeRight, latestResult);
        }

        for(var result: outtakeRightResults){
            Optional<EstimatedRobotPose> estimatedPose = CameraConstants.photonPoseEstimatorOuttakeRight.update(result);
            if(!estimatedPose.isEmpty()){
                calculateVisionMeasurement(estimatedPose.get(), 1);
            }
        }

        //intake
        CameraConstants.photonPoseEstimatorIntake.setReferencePose(getPose());
        var intakeResults = CameraConstants.photonCameraIntake.getAllUnreadResults();

        if(!intakeResults.isEmpty()){
            var latestResult = intakeResults.get(intakeResults.size()-1);
            m_driveState.setLatestPhotonVisionResult(CameraConstants.photonCameraIntake, latestResult);
        }

        for(var result: intakeResults){
            Optional<EstimatedRobotPose> estimatedPose = CameraConstants.photonPoseEstimatorIntake.update(result);
            if(!estimatedPose.isEmpty()){
                calculateVisionMeasurement(estimatedPose.get(), 2);
            }
        }

    }
  
    public void calculateVisionMeasurement(EstimatedRobotPose pose, int cameraId){
        double highestAmbiguity = 0;
        double maxTargetSize = 0;
        double xyStds = 0.5;
        double thetaStd = 0.5;
        double poseDistance = pose.estimatedPose.toPose2d().getTranslation().getDistance(this.getState().Pose.getTranslation());
        for(PhotonTrackedTarget target: pose.targetsUsed) {
            tagsUsed.add(new TrackedAprilTag(target.getFiducialId(), target.getArea(), this.getPose().getTranslation().getDistance(target.bestCameraToTarget.getTranslation().toTranslation2d()) , target.getPoseAmbiguity(), target.getYaw() , cameraId));
            
            tagPosesFieldRelative.add(new Pose3d(getPose())
                .transformBy(CameraConstants.allPhotonPoseEstimators[cameraId].getRobotToCameraTransform())
                .transformBy(target.getBestCameraToTarget()));
                
            if(target.getPoseAmbiguity() > highestAmbiguity){
                highestAmbiguity = target.getPoseAmbiguity();
            }
            if(target.area > maxTargetSize){
                maxTargetSize = target.area;
            }
        }
        //if the pose is too ambiguous, don't use it
        if(highestAmbiguity > 0.7){
            return;
        //if the target is large
        } else if (maxTargetSize > 4){
            // we're not moving, trust the pose
            if (this.getState().Speeds.vxMetersPerSecond + this.getState().Speeds.vyMetersPerSecond < 0.2){
                xyStds = 0.1;
                thetaStd = 0.1;  
            // new pose is close to the old pose, trust the pose
            } else if (poseDistance < 0.5){
                xyStds = 0.15;
                thetaStd = 0.15;
            }
        } else if (maxTargetSize > 2){
            // we're not moving, trust the pose
            if (this.getState().Speeds.vxMetersPerSecond + this.getState().Speeds.vyMetersPerSecond < 0.2){
                xyStds = 0.2;
                thetaStd = 0.2;
            // new pose is close to the old pose, trust the pose
                } else if (poseDistance < 0.5){
                xyStds = 0.25;
                thetaStd = 0.25;
            }
        } else if (highestAmbiguity < 0.3) {
            xyStds = 0.5;
            thetaStd = 0.5;            
        } else {
            xyStds = 0.8;
            thetaStd = 0.8;
        }

        //if you're spinning, bail.
        if(this.getState().Speeds.omegaRadiansPerSecond > Math.PI) {
            return;
        }

        //if you're rotating quickly, trust the pose less
        if (this.getState().Speeds.omegaRadiansPerSecond > 0.5){
            thetaStd = 0.99;
        }

        this.addVisionMeasurement(pose.estimatedPose.toPose2d(), Utils.getCurrentTimeSeconds() - 200.0/1000.0, VecBuilder.fill(xyStds, xyStds, thetaStd));
    }


}

