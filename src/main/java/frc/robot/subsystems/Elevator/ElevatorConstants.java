package frc.robot.subsystems.Elevator;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;

public class ElevatorConstants {
        //Motor
        public static final int kElevatorMotorLeaderId = 6;
        public static final int kElevatorMotorFollowerId = 7;
        public static final TalonFX ELEVATOR_LEADER_MOTOR = new TalonFX(kElevatorMotorLeaderId);
        public static final TalonFX ELEVATOR_FOLLOWER_MOTOR = new TalonFX(kElevatorMotorFollowerId);


        //Slot0Configs

        public static final double ELEVATOR_kV = 0;
        public static final double ELEVATOR_kS = 0;
        public static final double ELEVATOR_kP = 0;
        public static final double ELEVATOR_kI = 0;
        public static final double ELEVATOR_kD = 0;
        public static final double ELEVATOR_kG = 0;
        public static final GravityTypeValue ELEVATOR_GRAVITY = GravityTypeValue.Elevator_Static;

        public static Slot0Configs createSlot0Configs(){ 
            Slot0Configs slot = new Slot0Configs();
            slot.kV = ELEVATOR_kV;
            slot.kS = ELEVATOR_kS;
            slot.kP = ELEVATOR_kP;
            slot.kI = ELEVATOR_kI;
            slot.kD = ELEVATOR_kD;
            slot.kG = ELEVATOR_kG;
            slot.GravityType = ELEVATOR_GRAVITY;
           return slot; 
        }

        //SoftLimitConfig
        public static final double ELEVATOR_FORWARD_SOFT_LIMIT = 22.5;
        public static final double ELEVATOR_REVERSE_SOFT_LIMIT = 0.25;

        public static SoftwareLimitSwitchConfigs createSoftLimitConigs(){
            SoftwareLimitSwitchConfigs newConfigs = new SoftwareLimitSwitchConfigs();
            newConfigs.ForwardSoftLimitEnable = true;
            newConfigs.ReverseSoftLimitEnable = true;
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
        public static MotorOutputConfigs createLeaderMotorOutputConfigs(){
            MotorOutputConfigs newConfigs = new MotorOutputConfigs();
            newConfigs.Inverted = InvertedValue.Clockwise_Positive;
            return newConfigs;
        }

        public static MotorOutputConfigs createFollowerMotorOutputConfigs(){
            MotorOutputConfigs newConfigs = new MotorOutputConfigs();
            newConfigs.Inverted = InvertedValue.CounterClockwise_Positive;
            return newConfigs;
        }

        public enum FLOOR{
            GROUND(0),
            TROUGH(1),
            LEVEL_2(2),
            LEVEL_3(3),
            LEVEL_4(4);

            public final int height;
            FLOOR(int value){
                height = value;
            }
        }

        public static double POSITION_ERROR = 0.01;
}

