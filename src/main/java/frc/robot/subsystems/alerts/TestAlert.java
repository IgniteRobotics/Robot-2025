package frc.robot.subsystems.alerts;

import java.util.concurrent.atomic.AtomicBoolean;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.epilogue.Logged.Importance;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.robot.Robot;
import edu.wpi.first.wpilibj.RobotController;

@Logged
public class TestAlert {
  public static Alert test2 = new Alert("test alert", AlertType.kInfo);
  public static Boolean warning = false;
  private boolean flash = false;
  private double ts = ((int)RobotController.getFPGATime() / 500000) % 2;
  public void Testg(){
    if (warning == true){
      if(ts % 2 == 1){
       flash = true;
       } else if(ts % 2 == 0){
       flash = false;
       }
   }
  };     
}