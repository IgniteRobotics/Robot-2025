package frc.robot.subsystems.Elevator;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.hardware.TalonFX;

public class ElevatorConstants {
        //Motor
        public static final int kElevatorMotorId = 11;
        public static final TalonFX ELEVATOR_MOTOR = new TalonFX(kElevatorMotorId);


        //Slot0Configs

        public static final double ELEVATOR_kV = 0;
        public static final double ELEVATOR_kS = 0;
        public static final double ELEVATOR_kP = 0;
        public static final double ELEVATOR_kI = 0;
        public static final double ELEVATOR_kD = 0;

        public static Slot0Configs createSlot0Configs(){ 
            Slot0Configs slot = new Slot0Configs();
            slot.kV = ELEVATOR_kV;
           slot.kS = ELEVATOR_kS;
            slot.kP = ELEVATOR_kP;
            slot.kI = ELEVATOR_kI;
            slot.kD = ELEVATOR_kD;
           return slot; 
        }

        //SoftLimitConfig
        public static final double ELEVATOR_FORWARD_SOFT_LIMIT = 100;
        public static final double ELEVATOR_REVERSE_SOFT_LIMIT = 0;

        public static SoftwareLimitSwitchConfigs createSoftLimitConigs(){
            SoftwareLimitSwitchConfigs newConfigs = new SoftwareLimitSwitchConfigs();
            newConfigs.ForwardSoftLimitEnable = false;
            newConfigs.ReverseSoftLimitEnable = false;
            newConfigs.ForwardSoftLimitThreshold = ELEVATOR_FORWARD_SOFT_LIMIT;
             newConfigs.ReverseSoftLimitThreshold = ELEVATOR_REVERSE_SOFT_LIMIT;
                return newConfigs;
        }

        //MotionMagicConfigs
        public static MotionMagicConfigs createMotionMagicConfigs(){
            MotionMagicConfigs newConfigs = new MotionMagicConfigs();
            //TODO: ADD MORE IF NECESSARY
            return newConfigs;
        }

        //MotorConfigs
        public static MotorOutputConfigs createMotorOutputConfigs(){
        MotorOutputConfigs newConfigs = new MotorOutputConfigs();
        //TODO:ADD MORE IF NECESSARY
        return newConfigs;
        }
}

