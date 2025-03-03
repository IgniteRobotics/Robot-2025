// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.algae;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.ProximityParamsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Preferences;
import frc.robot.PreferenceTypes.DoublePreference;
import frc.robot.statemachines.RobotState;

@Logged
public class AlgaeCollector extends SubsystemBase {
  private final TalonFX m_algaeMotor;
  private final TalonFX m_wristMotor;
  private Slot0Configs m_algaeSlot0Configs;


  private Slot0Configs m_wristSlot0Configs;
  private SoftwareLimitSwitchConfigs m_wristSoftLimitConfigs;
  private TalonFXConfiguration m_wristTalonFXConfiguration;
  private MotorOutputConfigs m_wristMotorOutputConfigs;

  private final CANrange m_beambreak_algae;

  private double m_wristTargetPosition;

  public final static double WRIST_POSITION_ERROR = 0.1;


  

  private final RobotState m_robotState = RobotState.getInstance();
    
    
  /** Creates a new EndEffector. */
  public AlgaeCollector() {
    m_algaeMotor = new TalonFX(AlgaeCollectorConstants.kAlgaeMotorId);
    m_algaeMotor.getConfigurator().apply(AlgaeCollectorConstants.createAlgaeMotorOutputConfigs());
    m_algaeSlot0Configs = AlgaeCollectorConstants.createAlgaeMotorSlot0Configs();
    m_algaeMotor.getConfigurator().apply(m_algaeSlot0Configs);
    m_algaeMotor.getConfigurator().apply(AlgaeCollectorConstants.createCurrentLimitsConfigs());
  


    m_wristMotor = new TalonFX(AlgaeCollectorConstants.kWristMotorId);
    m_wristTalonFXConfiguration = AlgaeCollectorConstants.createWristTalonFXConfigs();
    m_wristMotor.getConfigurator().apply(m_wristTalonFXConfiguration);
    m_wristSlot0Configs = AlgaeCollectorConstants.createWristMotorSlot0Configs();
    m_wristMotor.getConfigurator().apply(m_wristSlot0Configs);
    m_wristSoftLimitConfigs = AlgaeCollectorConstants.createWristSoftLimitConfigs();
    m_wristMotorOutputConfigs = AlgaeCollectorConstants.createWristMotorOutputConfigs();
    m_wristMotor.getConfigurator().apply(m_wristMotorOutputConfigs);
    m_wristMotor.setPosition(0.282715);

    m_beambreak_algae = new CANrange(AlgaeCollectorConstants.kAlgaeBeamBreakId);
    
    configureCANrange();
  }

  public void configureCANrange(){
    ProximityParamsConfigs proximityParamsConfigs = new ProximityParamsConfigs();

    m_beambreak_algae.getConfigurator().refresh(proximityParamsConfigs);
    m_beambreak_algae.getConfigurator().apply(
      proximityParamsConfigs
        .withProximityThreshold(Units.Inches.of(1))
        .withProximityHysteresis(Units.Inches.of(.25))
      );
    
  }

  public void stopAlgaeMotor(){
    m_algaeMotor.stopMotor();
  }

  public void intakeAlgae(){
    m_algaeMotor.set(Preferences.algaeIntakePower.getValue());
  }

  public void outtakeAlgae(){
    m_algaeMotor.set(Preferences.algaeOuttakePower.getValue());
  }

  public void setWristPosition(double position){
    m_wristTargetPosition = position;
    m_wristMotor.setControl(new PositionVoltage(position).withSlot(0));
  }

  public void setWristPosition(DoublePreference position){
    setWristPosition(position.getValue());
  }

  public boolean isWristAtPosition(){
    return Math.abs(m_wristMotor.getPosition().getValue().magnitude() - m_wristTargetPosition)  <=  AlgaeCollectorConstants.WRIST_POSITION_ERROR; 
  }

  public void stow(){
    if(m_robotState.hasAlgae()){
      setWristPosition(Preferences.collectorStowPositionWithAlgae);
    }
    else{
      setWristPosition(Preferences.collectorStowPositionWithAlgae);
    }
  }

  @Logged
  public boolean seesAlgae(){
    return m_beambreak_algae.getIsDetected().getValue();
  }

  @Override
  public void periodic() {
    m_robotState.setHasAlgae(seesAlgae());
  }

}
