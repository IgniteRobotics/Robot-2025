// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.EndEffector;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class EndEffector extends SubsystemBase {

  private final TalonFX m_coralMotor;
  private final TalonFX m_algaeMotor;
  private final TalonFX m_wristMotor;



  /** Creates a new EndEffector. */
  public EndEffector() {
    m_coralMotor = new TalonFX(EndEffectorConstants.kCoralMotorId);
    m_algaeMotor = new TalonFX(EndEffectorConstants.kAlgaeMotorId);
    m_wristMotor = new TalonFX(EndEffectorConstants.kWristMotorId);
  }

  public void configureCoralMotor(){
      SoftwareLimitSwitchConfigs m_softLimitConfig = new SoftwareLimitSwitchConfigs();
      MotionMagicConfigs m_motionMagicConfigs = new MotionMagicConfigs();
      MotorOutputConfigs m_motorConfig = new MotorOutputConfigs();
      TalonFXConfiguration m_fxCfg = new TalonFXConfiguration();

      Slot0Configs slot = new Slot0Configs();
      slot.kV = EndEffectorConstants.CORAL_kV;
      slot.kS = EndEffectorConstants.CORAL_kS;
      slot.kP = EndEffectorConstants.CORAL_kP;
      slot.kI = EndEffectorConstants.CORAL_kI;
      slot.kD = EndEffectorConstants.CORAL_kD;
      m_coralMotor.getConfigurator().apply(slot);
    }

  public void configureAlgaeMotor(){
      SoftwareLimitSwitchConfigs m_softLimitConfig = new SoftwareLimitSwitchConfigs();
      MotionMagicConfigs m_motionMagicConfigs = new MotionMagicConfigs();
      MotorOutputConfigs m_motorConfig = new MotorOutputConfigs();
      TalonFXConfiguration m_fxCfg = new TalonFXConfiguration();

      Slot0Configs slot = new Slot0Configs();
      slot.kV = EndEffectorConstants.ALGAE_kV;
      slot.kS = EndEffectorConstants.ALGAE_kS;
      slot.kP = EndEffectorConstants.ALGAE_kP;
      slot.kI = EndEffectorConstants.ALGAE_kI;
      slot.kD = EndEffectorConstants.ALGAE_kD;

      m_algaeMotor.getConfigurator().apply(slot);
  }

  public void configureWristMotor(){
      SoftwareLimitSwitchConfigs m_softLimitConfig = new SoftwareLimitSwitchConfigs();
      MotionMagicConfigs m_motionMagicConfigs = new MotionMagicConfigs();
      MotorOutputConfigs m_motorConfig = new MotorOutputConfigs();
      TalonFXConfiguration m_fxCfg = new TalonFXConfiguration();

      Slot0Configs slot = new Slot0Configs();
      slot.kV = EndEffectorConstants.WRIST_kV;
      slot.kS = EndEffectorConstants.WRIST_kS;
      slot.kP = EndEffectorConstants.WRIST_kP;
      slot.kI = EndEffectorConstants.WRIST_kI;
      slot.kD = EndEffectorConstants.WRIST_kD;
      slot.kG = EndEffectorConstants.WRIST_kG;

      m_wristMotor.getConfigurator().apply(slot);

      double WRIST_FORWARD_SOFT_LIMIT = 100;
      double WRIST_REVERSE_SOFT_LIMIT = 0;

      SoftwareLimitSwitchConfigs newConfigs = new SoftwareLimitSwitchConfigs();
      newConfigs.ForwardSoftLimitEnable = false;
      newConfigs.ReverseSoftLimitEnable = false;
      newConfigs.ForwardSoftLimitThreshold = WRIST_FORWARD_SOFT_LIMIT;
      newConfigs.ReverseSoftLimitThreshold = WRIST_REVERSE_SOFT_LIMIT;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
