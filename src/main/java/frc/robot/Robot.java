// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.sql.Time;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.epilogue.logging.EpilogueBackend;
import edu.wpi.first.epilogue.logging.FileBackend;
import edu.wpi.first.epilogue.logging.NTEpilogueBackend;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.PreferenceTypes.DoublePreference;
import frc.robot.subsystems.alerts.Alerts;

@Logged
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  private final boolean kUseLimelight = false;

  private final RobotState m_robotState = RobotState.getInstance();

  private boolean hasAlliance  = false;

  private boolean Idontcarewhatitscalled = false;

  public Robot() {
    m_robotContainer = new RobotContainer();
    DataLogManager.start();
    Epilogue.bind(this);
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
  public void disabledPeriodic() {
    if (DriverStation.isFMSAttached() && Idontcarewhatitscalled == false){
      Epilogue.configure(config -> {
        if (Robot.isSimulation()) {
            config.backend = EpilogueBackend.multi(
                    new FileBackend(DataLogManager.getLog())
            );
        }
    });
    Epilogue.bind(this);
    Idontcarewhatitscalled = true;
    }
  }

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
    Idontcarewhatitscalled = true;
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
    Idontcarewhatitscalled = true;
    }
  

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
  //call the alert subsystem
  Alerts example = new Alerts();
  @Override
  public void simulationPeriodic() {
        //set the alert
        example.example.set(DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red);
        //set the warning flash if desired
        if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red){
          Alerts.flash = true;
        }
  }
}
