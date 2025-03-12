package frc.robot.subsystems.alerts;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.RobotController;

public class Alerts {
  //Alerts go here
  public Alert example = new Alert("example alert", AlertType.kInfo);

  //Flashing epiloge warning light code
  public static boolean flash = false;
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
}
