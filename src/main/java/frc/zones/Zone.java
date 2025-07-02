package frc.zones;

import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class Zone{
    public String name;
    public Double maxSpeed;
    public Double maxRotation;
    public Integer targetID;
    public Double heading;

    public void setMaxSpeed(double speed){
        maxSpeed = speed;
    }

    public void setMaxRotation(double rot){
        maxRotation = rot;
    }

    public void setTargetID(int id){
        targetID = id;
    }

    public void setHeading(double head){
        heading = head;
    }

    public void setName(String aName){
        name = aName;
    }

}

    

