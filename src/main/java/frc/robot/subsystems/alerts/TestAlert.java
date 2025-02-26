package frc.robot.subsystems.alerts;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.RobotController;

public class TestAlert {
  public boolean warning = false;
  private boolean flash = false;
  private double ts = ((int)RobotController.getFPGATime() / 500000) % 2;
  public Test(alert test){
    alert = test;
    Alert test2 = new Alert("test alert", AlertType.kInfo);
    if (warning == true){
      if(ts % 2 == 1){
       flash = true;
       } else if(ts % 2 == 0){
       flash = false;
       }
   }
  };     
}