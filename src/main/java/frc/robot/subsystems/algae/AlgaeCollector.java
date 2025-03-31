// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.algae;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.ProximityParamsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Preferences;
import frc.robot.PreferenceTypes.DoublePreference;
import frc.robot.statemachines.AlgaeState;

@Logged
public class AlgaeCollector extends SubsystemBase {
  private final TalonFX m_algaeMotor;
  private final TalonFX m_wristMotor;
  private Slot0Configs m_algaeSlot0Configs;


  private Slot0Configs m_wristSlot0Configs;
  private SoftwareLimitSwitchConfigs m_wristSoftLimitConfigs;
  private TalonFXConfiguration m_wristTalonFXConfiguration;
  private MotorOutputConfigs m_wristMotorOutputConfigs;
  private MotionMagicConfigs m_wristMMConfigs;

  private double m_wristTargetPosition;

  public final static double WRIST_POSITION_ERROR = 0.1;

  private final AlgaeState m_algaeState = AlgaeState.getInstance();
    
    
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
    m_wristMMConfigs = AlgaeCollectorConstants.createWristMotionMagicConfigs();
    m_wristMotor.getConfigurator().apply(m_wristMMConfigs);
    m_wristMotor.setPosition(0.282715);

  }

  public void stopAlgaeMotor(){
    m_algaeMotor.set(0);
  }

  public void stopWristMotor(){
    m_wristMotor.stopMotor();
  }

  public void intakeAlgae(){
    m_algaeMotor.set(Preferences.algaeIntakePower.getValue());
  }

  public void holdAlgae(){
    m_algaeMotor.set(Preferences.algaeHoldPower.getValue());
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

  public void setToIntakePosition(){
    setWristPosition(AlgaeCollectorConstants.WRIST.REEF.angle);
  }

  public void stow(){
    setWristPosition(Preferences.collectorStowPosition);
  }

  @Logged
  public boolean seesAlgae(){
    return false;
  }

  public boolean checkCurrentSpike(){
    double supplyCurrent = m_algaeMotor.getSupplyCurrent().getValueAsDouble();
    if (supplyCurrent > AlgaeCollectorConstants.createCurrentLimitsConfigs().StatorCurrentLimit) {
      return true;
    } else {
      return false;
    }
  }

  public void setWristMotionMagic(DoublePreference CV, DoublePreference A, DoublePreference J){
    m_wristMotor.getConfigurator().refresh(m_wristMMConfigs);
    m_wristMMConfigs.MotionMagicCruiseVelocity = CV.getValue();
    m_wristMMConfigs.MotionMagicAcceleration = A.getValue();
    m_wristMMConfigs.MotionMagicJerk = J.getValue();
    m_wristMotor.getConfigurator().apply(m_wristMMConfigs);
  }

  @Override
  public void periodic() {
    SmartDashboard.putString("Algae Target", m_algaeState.getAlgaeTargetName());
    m_algaeState.setHasAlgae(seesAlgae());
  }

  //Voltage, Current, Temperature

  /*********Logging Motors*************/

  @Logged(name = "Wrist Motor Position", importance = Importance.CRITICAL)
  public double getWristPosition(){
    return m_wristMotor.getPosition().getValueAsDouble();
  }

  @Logged(name = "Wrist Motor Voltage", importance = Importance.CRITICAL)
  public double getWristVoltage(){
    return m_wristMotor.getMotorVoltage().getValueAsDouble();
  }

  @Logged(name = "Wrist Motor Current", importance = Importance.CRITICAL)
  public double getWristCurrent(){
    return m_wristMotor.getSupplyCurrent().getValueAsDouble();
  }

  @Logged(name = "Wrist Motor Temperature", importance = Importance.CRITICAL)
  public double getWristTemperature(){
    return m_wristMotor.getDeviceTemp().getValueAsDouble();
  }

  @Logged(name = "Algae Motor Voltage", importance = Importance.CRITICAL)
  public double getAlgaeVoltage(){
    return m_algaeMotor.getMotorVoltage().getValueAsDouble();
  }

  @Logged(name = "Algae Motor Current", importance = Importance.CRITICAL)
  public double getAlgaeCurrent(){
    return m_algaeMotor.getSupplyCurrent().getValueAsDouble();
  }

  @Logged(name = "Algae Motor Temperature", importance = Importance.CRITICAL)
  public double getAlgaeTemperature(){
    return m_algaeMotor.getDeviceTemp().getValueAsDouble();
  }

}
