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
  public static boolean flash = false;
  public Alert test2 = new Alert("test alert", AlertType.kInfo);
  private boolean Warning = false;
  public void Testg(){
    if (flash == true){
      if(((int)RobotController.getFPGATime() / 500000) % 2 == 1){
       Warning = true;
       } else if(((int)RobotController.getFPGATime() / 500000) % 2 == 0){
       Warning = false;
       }
      flash = false;
   }
  };
  //public boolean Testh(){
  //  return flash;
  //}     
}