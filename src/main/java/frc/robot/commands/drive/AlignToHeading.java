package frc.robot.commands.drive;

import java.util.Optional;
import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Preferences;
import frc.robot.PreferenceTypes.DoublePreference;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.drive.PhotonCameraWrapper;
import frc.robot.subsystems.drive.PhotonCameraWrapper.TargetInfo;

public class AlignToHeading extends Command {
  private final CommandSwerveDrivetrain m_drive;
  private final PhotonCameraWrapper m_camera;
  private final Supplier<Double> m_driveX;
  private final Supplier<Double> m_heading;
  PIDController rotationController;

  
  public AlignToHeading(CommandSwerveDrivetrain drive, PhotonCameraWrapper camera, Supplier<Double> supplier, Supplier<Double> heading) {
    m_drive = drive;
    m_camera = camera;
    m_driveX = supplier;
    m_heading = heading;
    addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    rotationController = new PIDController(Preferences.turnToTarget_rotKP.getValue(), 0, Preferences.turnToTarget_rotKD.getValue());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double rotation = rotationController.calculate(m_drive.getYaw(), m_heading.get());
    m_drive.driveRobotCentric(m_driveX.get(), 0, rotation);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
