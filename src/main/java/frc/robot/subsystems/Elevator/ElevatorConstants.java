package frc.robot.subsystems.Elevator;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.ctre.phoenix6.controls.Follower;

public class ElevatorConstants {
        //Motor
        public static final int kElevatorMotorLeaderId = 6;
        public static final int kElevatorMotorFollowerId = 7;
        public static final TalonFX ELEVATOR_LEADER_MOTOR = new TalonFX(kElevatorMotorLeaderId);
        public static final TalonFX ELEVATOR_FOLLOWER_MOTOR = new TalonFX(kElevatorMotorFollowerId){{
            setControl(new Follower(kElevatorMotorLeaderId, true));
        }};


        //Slot0Configs

        //with coral
        public static final double ELEVATOR_kV = 0;
        public static final double ELEVATOR_kS = 0.5;
        public static final double ELEVATOR_kP = 7;
        public static final double ELEVATOR_kI = 0;
        public static final double ELEVATOR_kD = 0.25;
        public static final double ELEVATOR_kG = 0.5;
        public static final GravityTypeValue ELEVATOR_GRAVITY = GravityTypeValue.Elevator_Static;
        public static final StaticFeedforwardSignValue ELEVATOR_FEEDFORWARD = StaticFeedforwardSignValue.UseClosedLoopSign;

        public static Slot0Configs createSlot0Configs(){ 
            Slot0Configs slot = new Slot0Configs();
            slot.kV = ELEVATOR_kV;
            slot.kS = ELEVATOR_kS;
            slot.kP = ELEVATOR_kP;
            slot.kI = ELEVATOR_kI;
            slot.kD = ELEVATOR_kD;
            slot.kG = ELEVATOR_kG;
            slot.GravityType = ELEVATOR_GRAVITY;
            slot.StaticFeedforwardSign = ELEVATOR_FEEDFORWARD;
            return slot; 
        }

        //SoftLimitConfig
        public static final double ELEVATOR_FORWARD_SOFT_LIMIT = 26;
        public static final double ELEVATOR_REVERSE_SOFT_LIMIT = 0.04;

        public static SoftwareLimitSwitchConfigs createSoftLimitConigs(){
            SoftwareLimitSwitchConfigs newConfigs = new SoftwareLimitSwitchConfigs();
            newConfigs.ForwardSoftLimitEnable = true;
            newConfigs.ReverseSoftLimitEnable = true;
            newConfigs.ForwardSoftLimitThreshold = ELEVATOR_FORWARD_SOFT_LIMIT;
             newConfigs.ReverseSoftLimitThreshold = ELEVATOR_REVERSE_SOFT_LIMIT;
                return newConfigs;
        }

        //MotionMagicConfigs
        public static final double ELEVATOR_MM_JERK = 2000;
        public static final double ELEVATOR_MM_ACCEL = 200;
        public static final double ELEVATOR_MM_CRUISE_VELOCITY = 100;
        public static MotionMagicConfigs createMotionMagicConfigs(){
            MotionMagicConfigs newConfigs = new MotionMagicConfigs();
            newConfigs.MotionMagicJerk = ELEVATOR_MM_JERK;
            newConfigs.MotionMagicAcceleration = ELEVATOR_MM_ACCEL;
            newConfigs.MotionMagicCruiseVelocity = ELEVATOR_MM_CRUISE_VELOCITY;
            return newConfigs;
        }

        //MotorConfigs
        public static MotorOutputConfigs createLeaderMotorOutputConfigs(){
            MotorOutputConfigs newConfigs = new MotorOutputConfigs();
            newConfigs.Inverted = InvertedValue.Clockwise_Positive;
            newConfigs.NeutralMode = NeutralModeValue.Brake;
            return newConfigs;
        }

        public static MotorOutputConfigs createFollowerMotorOutputConfigs(){
            MotorOutputConfigs newConfigs = new MotorOutputConfigs();
            newConfigs.Inverted = InvertedValue.CounterClockwise_Positive;
            newConfigs.NeutralMode = NeutralModeValue.Brake;
            return newConfigs;
        }

        public enum FLOOR{
            GROUND(0.25),
            TROUGH(4.61),
            LEVEL_2(8.40),
            LEVEL_3(16.13),
            LEVEL_4(25.5),
            HP(.25);

            public final double position;
            FLOOR(double value){
                position = value;
            }
        }

        public enum ALGAE{
            PROCESSOR(1),
            LOW_REEF(8.6),
            HIGH_REEF(14.9),
            BARGE(25.5);

            public final double height;
            ALGAE(double value){
                height = value;
            }
        }

        public final static double POSITION_ERROR = 0.25;
}

