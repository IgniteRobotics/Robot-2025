// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;

public class ClimberConstants {

        //Motor
        public static final int kEclimberMotorLeaderId = 14;
        public static final int kEclimberMotorFollowerId = 15;
        public static final TalonFX CLIMBER_LEADER_MOTOR = new TalonFX(kEclimberMotorLeaderId);
        public static final TalonFX CLIMBER_FOLLOWER_MOTOR = new TalonFX(kEclimberMotorFollowerId);


        //Slot0Configs

        public static final double CLIMBER_kV = 0;
        public static final double CLIMBER_kS = 0;
        public static final double CLIMBER_kP = 0;
        public static final double CLIMBER_kI = 0;
        public static final double CLIMBER_kD = 0;
        public static final double CLIMBER_kG = 0;
        public static final GravityTypeValue CLIMBER_GRAVITY = GravityTypeValue.Elevator_Static;

        public static Slot0Configs createSlot0Configs(){ 
            Slot0Configs slot = new Slot0Configs();
            slot.kV = CLIMBER_kV;
            slot.kS = CLIMBER_kS;
            slot.kP = CLIMBER_kP;
            slot.kI = CLIMBER_kI;
            slot.kD = CLIMBER_kD;
            slot.kG = CLIMBER_kG;
            slot.GravityType = CLIMBER_GRAVITY;
           return slot; 
        }

        //SoftLimitConfig
        public static final double CLIMBER_FORWARD_SOFT_LIMIT = 100;
        public static final double CLIMBER_REVERSE_SOFT_LIMIT = 0;

        public static SoftwareLimitSwitchConfigs createSoftLimitConigs(){
            SoftwareLimitSwitchConfigs newConfigs = new SoftwareLimitSwitchConfigs();
            newConfigs.ForwardSoftLimitEnable = false;
            newConfigs.ReverseSoftLimitEnable = false;
            newConfigs.ForwardSoftLimitThreshold = CLIMBER_FORWARD_SOFT_LIMIT;
             newConfigs.ReverseSoftLimitThreshold = CLIMBER_REVERSE_SOFT_LIMIT;
                return newConfigs;
        }

        //MotionMagicConfigs
        public static final double CLIMBER_MM_JERK = 101;
        public static final double CLIMBER_MM_ACCEL = 10;
        public static final double CLIMBER_MM_CRUISE_VELOCITY = 5;
        public static final double CLIMBER_MM_kV = 0.1;
        public static final double CLIMBER_MM_kA = 0.1;
        public static MotionMagicConfigs createMotionMagicConfigs(){
            MotionMagicConfigs newConfigs = new MotionMagicConfigs();
            newConfigs.MotionMagicJerk = CLIMBER_MM_JERK;
            newConfigs.MotionMagicAcceleration = CLIMBER_MM_ACCEL;
            newConfigs.MotionMagicCruiseVelocity = CLIMBER_MM_CRUISE_VELOCITY;
            newConfigs.MotionMagicExpo_kV = CLIMBER_MM_kV;
            newConfigs.MotionMagicExpo_kA = CLIMBER_MM_kA;
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


}