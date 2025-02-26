// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.PreferenceTypes.DoublePreference;
import frc.robot.util.DeviceFinder;

@Logged
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  private CommandScheduler m_Scheduler;

  private final boolean kUseLimelight = false;

  private final RobotState m_robotState = RobotState.getInstance();

  private boolean hasAlliance  = false;

  @NotLogged
  private DeviceFinder devFinder = new DeviceFinder();

  @NotLogged
  Alert canDeviceAlert = new Alert("TEST MODE","", AlertType.kInfo);

  public Robot() {
    m_robotContainer = new RobotContainer();
    m_Scheduler = CommandScheduler.getInstance();
    DataLogManager.start();
    // Record both DS control and joystick data
    DriverStation.startDataLog(DataLogManager.getLog());

    Epilogue.bind(this);

    StringLogEntry metaData = new StringLogEntry(DataLogManager.getLog(), "MetaData");
    metaData.append("Project Name: " + BuildConstants.MAVEN_NAME);
    metaData.append("Build Date: " + BuildConstants.BUILD_DATE);
    metaData.append("Commit Hash: " + BuildConstants.GIT_SHA);
    metaData.append("Git Date: " + BuildConstants.GIT_DATE);
    metaData.append("Git Branch: " + BuildConstants.GIT_BRANCH);
    switch (BuildConstants.DIRTY) {
      case 0:
        metaData.append("GitDirty: " + "All changes commited");
        break;
      case 1:
        metaData.append("GitDirty: " + "Uncomitted changes");
        break;
      default:
        metaData.append("GitDirty: " + "Unknown");
        break;
    }
  }

  private void getAllianceInfo(){
    if (DriverStation.getAlliance().isPresent()) {
      hasAlliance = true;
      if (DriverStation.getAlliance().get() == Alliance.Red){
        m_robotState.setGrid(Alliance.Red);
      } else {
        m_robotState.setGrid(Alliance.Blue);
      }
    }
  }

  @Override
  public void robotPeriodic() {
    if (!hasAlliance) {getAllianceInfo();}

    CommandScheduler.getInstance().run();

    /*
     * This example of adding Limelight is very simple and may not be sufficient for on-field use.
     * Users typically need to provide a standard deviation that scales with the distance to target
     * and changes with number of tags available.
     *
     * This example is sufficient to show that vision integration is possible, though exact implementation
     * of how to use vision should be tuned per-robot and to the team's specification.
     */
    if (kUseLimelight) {
      var llMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");
      if (llMeasurement != null) {
        m_robotContainer.drivetrain.addVisionMeasurement(llMeasurement.pose, Utils.fpgaToCurrentTime(llMeasurement.timestampSeconds));
      }
    }
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
    ArrayList<String> devices = devFinder.find();
    String text = "Devices Found:";
    for (String device : devices) {
      text = text + "\n" + device;
    }
    canDeviceAlert.setText(text);
    canDeviceAlert.set(true);
  }

  @Override
  public void testPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void testExit() {}
  
  @Override
  public void simulationPeriodic() {

  }
}
