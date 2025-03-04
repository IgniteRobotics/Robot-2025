
package frc.robot.commands.auton;

import java.util.Optional;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Preferences;
import frc.robot.PreferenceTypes.DoublePreference;
import frc.robot.statemachines.AllianceState;
import frc.robot.subsystems.drive.CameraConstants;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.drive.PhotonCameraWrapper;
import frc.robot.subsystems.drive.PhotonCameraWrapper.TargetInfo;

public class AutonAlignToReef extends Command {
  private final CommandSwerveDrivetrain m_drive;
  private final int m_cameraId;
  private final AllianceState m_allianceState = AllianceState.getInstance();
  private boolean closeEnough = false;
  PhotonCameraWrapper m_pcw;
  

  PIDController rotationController;
  PIDController driveYController;
  PIDController driveXController;
  AprilTagFieldLayout aprilTags = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

  /** Creates a new AlignToTarget. */
  public AutonAlignToReef(CommandSwerveDrivetrain drive,  PhotonCameraWrapper pcw, int cameraID){
    m_drive = drive;
    m_cameraId = cameraID;
    m_pcw = pcw;
    addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    rotationController = new PIDController(Preferences.alignRotKP.get(), 0, Preferences.alignRotKD.get());
    driveYController = new PIDController(Preferences.alignDriveYKP.get(), 0, Preferences.alignDriveYKD.get());
    driveXController = new PIDController(Preferences.alignDriveXKP.get(), 0, Preferences.alignDriveXKD.get());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    closeEnough = true; 

    Optional<TargetInfo> targeting = m_pcw.seekOuttakeTargets(m_allianceState.getReefTags(), m_cameraId);
    double rotation;
    double driveX;
    double driveY;

    //TODO: Add constant for 0.1
    if(targeting.isPresent()){
      double targetHeading = Math.toDegrees(aprilTags.getTagPose(targeting.get().getTagId()).get().getRotation().rotateBy(new Rotation3d(0,0,Math.PI)).getZ());

      if(Math.abs(m_drive.getYaw() - targetHeading) > 0.1){
        closeEnough = false;
      }

      rotation = rotationController.calculate(m_drive.getYaw(), targetHeading);
      SmartDashboard.putNumber("Alignment/Data/Heading", targetHeading);

      double offset = CameraConstants.offsetToBumper.get(targeting.get().getCameraName());

      if(Math.abs(targeting.get().getDistance() - offset) > 0.1){
        closeEnough = false;
      }

      driveX = driveXController.calculate(targeting.get().getDistance(), offset);
      SmartDashboard.putNumber("Alignment/Data/Distance", targeting.get().getDistance());
      
      //TODO: Add Constant for -1
      if(Math.abs(targeting.get().getDistance() - (-1)) > 0.1){
        closeEnough = false;
      }
      driveY = -driveYController.calculate(targeting.get().getYaw(), -1);
      SmartDashboard.putNumber("Alignment/Data/Yaw", targeting.get().getYaw());
    }

    else{
      rotation = 0;
      driveX = 0;
      driveY = 0;
    }
  

    SmartDashboard.putNumber("Alignment/Power/rotation", rotation);
    SmartDashboard.putNumber("Alignment/Power/driveX", driveX);
    SmartDashboard.putNumber("Alignment/Power/driveY", driveY);
    
    m_drive.driveRobotCentric(driveX, driveY, rotation);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return closeEnough;
  }
}
