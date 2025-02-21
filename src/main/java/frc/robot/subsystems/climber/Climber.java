// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.PreferenceTypes.DoublePreference;

@Logged
public class Climber implements Subsystem {
    private final TalonFX m_climberMotorLeader;
    private final TalonFX m_climberMotorFollower;

  private Slot0Configs m_Slot0Configs = new Slot0Configs();
  
  private SoftwareLimitSwitchConfigs m_softLimitConfig = new SoftwareLimitSwitchConfigs();

  private MotionMagicConfigs m_motionMagicConfigs = new MotionMagicConfigs();

  private MotorOutputConfigs m_leaderMotorConfig = new MotorOutputConfigs();

  private MotorOutputConfigs m_followerMotorConfig = new MotorOutputConfigs();

  private TalonFXConfiguration m_fxCfg = new TalonFXConfiguration();

  public MotionMagicVoltage m_MMPosition =   new MotionMagicVoltage(0);

  //logged stuff
  @Logged(name = "Target Position", importance = Importance.CRITICAL)
  private double m_targetPosition;

  /** Creates a new Climber. */
  public Climber() {
    m_climberMotorLeader = ClimberConstants.CLIMBER_LEADER_MOTOR; 
    m_climberMotorFollower = ClimberConstants.CLIMBER_FOLLOWER_MOTOR;

    m_Slot0Configs = ClimberConstants.createSlot0Configs(); 
    m_climberMotorLeader.getConfigurator().apply(m_Slot0Configs);
    m_climberMotorFollower.getConfigurator().apply(m_Slot0Configs);

    m_softLimitConfig = ClimberConstants.createSoftLimitConigs(); 
    m_climberMotorLeader.getConfigurator().apply(m_softLimitConfig);
    m_climberMotorFollower.getConfigurator().apply(m_softLimitConfig);

    m_motionMagicConfigs = ClimberConstants.createMotionMagicConfigs();
    m_climberMotorLeader.getConfigurator().apply(m_motionMagicConfigs);
    m_climberMotorFollower.getConfigurator().apply(m_motionMagicConfigs);

    m_leaderMotorConfig = ClimberConstants.createLeaderMotorOutputConfigs();
    m_climberMotorLeader.getConfigurator().apply(m_leaderMotorConfig);

    m_followerMotorConfig = ClimberConstants.createFollowerMotorOutputConfigs();
    m_climberMotorFollower.getConfigurator().apply(m_followerMotorConfig);

  }

  @NotLogged
  public void setPositionRevolutions(double position) {
    m_targetPosition = position;
    m_climberMotorLeader.setControl(m_MMPosition.withPosition(position).withSlot(0));
    m_climberMotorFollower.setControl(m_MMPosition.withPosition(position).withSlot(0));
  }
  
  @NotLogged
  public void setPositionRevolutions(DoublePreference position){
    setPositionRevolutions(position.get());
  }

  @Logged
  public double getPosition(){
    return m_climberMotorLeader.getPosition().getValueAsDouble();
  }


  @Override
  public void periodic() {
    
  }

  @Override
  public void simulationPeriodic() {

  }

  public void setClimberPID(DoublePreference P, DoublePreference D, DoublePreference I, DoublePreference G){
    m_Slot0Configs = new Slot0Configs();

    m_climberMotorLeader.getConfigurator().refresh(m_Slot0Configs);

    m_Slot0Configs.withKP(P.getValue()).withKD(D.getValue()).withKI(I.getValue()).withKG(G.getValue());

    m_climberMotorLeader.getConfigurator().apply(m_Slot0Configs);
    m_climberMotorFollower.getConfigurator().apply(m_Slot0Configs);
  }

 public void setClimberMotionMagic(DoublePreference CV, DoublePreference A, DoublePreference J, DoublePreference kV, DoublePreference kA){
    m_motionMagicConfigs = new MotionMagicConfigs();
    m_climberMotorLeader.getConfigurator().refresh(m_motionMagicConfigs);

    m_motionMagicConfigs.withMotionMagicCruiseVelocity(CV.getValue()).withMotionMagicAcceleration(A.getValue()).withMotionMagicJerk(J.getValue())
      .withMotionMagicExpo_kV(kV.getValue()).withMotionMagicExpo_kA(kA.getValue());

    m_climberMotorLeader.getConfigurator().apply(m_motionMagicConfigs);
    m_climberMotorFollower.getConfigurator().apply(m_motionMagicConfigs);
  }

  @Logged(name = "Actual kP", importance = Importance.CRITICAL)
  public double getClimberkP(){
    m_Slot0Configs = new Slot0Configs();
    m_climberMotorLeader.getConfigurator().refresh(m_Slot0Configs);
    return m_Slot0Configs.kP;
  }

  @Logged(name = "Actual kD", importance = Importance.CRITICAL)
  public double getClimberkD(){
    m_Slot0Configs = new Slot0Configs();
    m_climberMotorLeader.getConfigurator().refresh(m_Slot0Configs);
    return m_Slot0Configs.kD;
  }

  @Logged(name = "Actual kI", importance = Importance.CRITICAL)
  public double getClimberkI(){
    m_Slot0Configs = new Slot0Configs();
    m_climberMotorLeader.getConfigurator().refresh(m_Slot0Configs);
    return m_Slot0Configs.kI;
  }

  @Logged(name = "Actual kG", importance = Importance.CRITICAL)
  public double getClimberkG(){
    m_Slot0Configs = new Slot0Configs();
    m_climberMotorLeader.getConfigurator().refresh(m_Slot0Configs);
    return m_Slot0Configs.kG;
  }

  @Logged(name = "Voltage", importance = Importance.CRITICAL)
  public double getVoltage(){
    return m_climberMotorLeader.getMotorVoltage().getValueAsDouble();
  }

  @Logged(name = "Current", importance = Importance.CRITICAL)
  public double getCurrent(){
    return m_climberMotorLeader.getStatorCurrent().getValueAsDouble();
  }


}