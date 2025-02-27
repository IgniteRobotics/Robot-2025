// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.EndEffector;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.ProximityParamsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Preferences;
import frc.robot.RobotState;
import frc.robot.PreferenceTypes.DoublePreference;
import frc.robot.subsystems.Elevator.ElevatorConstants;

@Logged
public class EndEffector extends SubsystemBase {

  private final TalonFX m_coralMotor;
  private final TalonFX m_algaeMotor;
  private final TalonFX m_wristMotor;

  private Slot0Configs m_coralSlot0Configs;
  private MotorOutputConfigs m_coralMotorOutputConfigs;

  private Slot0Configs m_algaeSlot0Configs;

  private Slot0Configs m_wristSlot0Configs;
  private SoftwareLimitSwitchConfigs m_wristSoftLimitConfigs;
  private TalonFXConfiguration m_wirstTalonFXConfiguration;

  private final CANrange m_beambreak_enter;
  private final CANrange m_beambreak_prep;


  private final RobotState m_robotState = RobotState.getInstance();

  /** Creates a new EndEffector. */
  public EndEffector() {
    m_coralMotor = new TalonFX(EndEffectorConstants.kCoralMotorId);

    m_coralSlot0Configs = EndEffectorConstants.createCoralMotorSlot0Configs();
    m_coralMotor.getConfigurator().apply(m_coralSlot0Configs);
    m_coralMotorOutputConfigs = EndEffectorConstants.createCoralMotorOutputConfigs();
    m_coralMotor.getConfigurator().apply(m_coralMotorOutputConfigs);

    m_algaeMotor = new TalonFX(EndEffectorConstants.kAlgaeMotorId);
    m_algaeSlot0Configs = EndEffectorConstants.createAlgaeMotorSlot0Configs();


    m_wristMotor = new TalonFX(EndEffectorConstants.kWristMotorId);
    m_wristSlot0Configs = EndEffectorConstants.createWirstMotorSlot0Configs();
    m_wristMotor.getConfigurator().apply(m_wristSlot0Configs);
    m_wristSoftLimitConfigs = EndEffectorConstants.createWristSoftLimitConfigs();
    m_wirstTalonFXConfiguration = EndEffectorConstants.createWristTalonFXConfigs();
    m_wristMotor.getConfigurator().apply(m_wirstTalonFXConfiguration);


    m_beambreak_enter = new CANrange(EndEffectorConstants.kEnterBeamBreakId);
    m_beambreak_prep = new CANrange(EndEffectorConstants.kPrepBeamBreakId);
    
    configureCANrange();
  }

  public void configureCANrange(){
    ProximityParamsConfigs proximityParamsConfigs = new ProximityParamsConfigs();
    m_beambreak_enter.getConfigurator().refresh(proximityParamsConfigs);
    m_beambreak_enter.getConfigurator().apply(
      proximityParamsConfigs
        .withProximityThreshold(Units.Inches.of(1))
        .withProximityHysteresis(Units.Inches.of(1.25))
        );
    
    m_beambreak_prep.getConfigurator().refresh(proximityParamsConfigs);
    m_beambreak_prep.getConfigurator().apply(
      proximityParamsConfigs
        .withProximityThreshold(Units.Inches.of(1))
        .withProximityHysteresis(Units.Inches.of(1.25))
      );
   
  }

  public void outtakeCoral(){
    m_coralMotor.set(Preferences.endEffectorCoralOuttakePower.getValue());
    }

  public void intakeCoral(){
    m_coralMotor.set(Preferences.endEffectorCoralIntakePower.getValue());
  }

  public void stopCoralMotor(){
    m_coralMotor.stopMotor();
  }

  public void setWristPosition(double position){
    m_wristMotor.setControl(new PositionVoltage(position).withSlot(0));
    /* 
    if(!m_robotState.hasAlgae())
      m_wristMotor.setControl(new PositionVoltage(position).withSlot(0));
    else
      m_wristMotor.setControl(new PositionVoltage(position).withSlot(1));
      */
  }

  public void setWristPosition(DoublePreference position){
    setWristPosition(position.getValue());
  }

  public void stow(){
    if(m_robotState.hasAlgae()){
      setWristPosition(Preferences.endEffectorStowPositionWithAlgae);
    }
    else{
      setWristPosition(Preferences.endEffectorStowPositionWithAlgae);
    }
  }

  @Logged
  public boolean seesCoralEnter(){
    return m_beambreak_enter.getIsDetected().getValue();
  }

  @Logged
  public boolean coralPreped(){
    return m_beambreak_prep.getIsDetected().getValue();
  }

  @Logged
  public boolean seesAlgae(){
    return false;
  }

  @Override
  public void periodic() {
    m_robotState.setHasCoral(!seesCoralEnter() && coralPreped());
    m_robotState.setHasAlgae(seesAlgae());
  }

}
