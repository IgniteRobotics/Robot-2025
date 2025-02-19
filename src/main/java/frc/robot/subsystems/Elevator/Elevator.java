package frc.robot.subsystems.Elevator;

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
public class Elevator implements Subsystem {
    private final TalonFX m_elevatorMotorLeader;
    private final TalonFX m_elevatorMotorFollower;

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
  public Elevator() {
    m_elevatorMotorLeader = ElevatorConstants.ELEVATOR_LEADER_MOTOR; 
    m_elevatorMotorFollower = ElevatorConstants.ELEVATOR_FOLLOWER_MOTOR;

    m_Slot0Configs = ElevatorConstants.createSlot0Configs(); 
    m_elevatorMotorLeader.getConfigurator().apply(m_Slot0Configs);
    m_elevatorMotorFollower.getConfigurator().apply(m_Slot0Configs);

    m_softLimitConfig = ElevatorConstants.createSoftLimitConigs(); 
    m_elevatorMotorLeader.getConfigurator().apply(m_softLimitConfig);
    m_elevatorMotorFollower.getConfigurator().apply(m_softLimitConfig);

    m_motionMagicConfigs = ElevatorConstants.createMotionMagicConfigs();
    m_elevatorMotorLeader.getConfigurator().apply(m_motionMagicConfigs);
    m_elevatorMotorFollower.getConfigurator().apply(m_motionMagicConfigs);

    m_leaderMotorConfig = ElevatorConstants.createLeaderMotorOutputConfigs();
    m_elevatorMotorLeader.getConfigurator().apply(m_leaderMotorConfig);

    m_followerMotorConfig = ElevatorConstants.createFollowerMotorOutputConfigs();
    m_elevatorMotorFollower.getConfigurator().apply(m_followerMotorConfig);

  }

  @NotLogged
  public void setPositionRevolutions(double position) {
    m_targetPosition = position;
    m_elevatorMotorLeader.setControl(m_MMPosition.withPosition(position).withSlot(0));
  }
  
  @NotLogged
  public void setPositionRevolutions(DoublePreference position){
    setPositionRevolutions(position.get());
  }

  @Logged
  public double getPosition(){
    return m_elevatorMotorLeader.getPosition().getValueAsDouble();
  }

  public boolean atSetpoint(double position){
    return((getPosition() - ElevatorConstants.POSITION_ERROR < position) && (getPosition()+ElevatorConstants.POSITION_ERROR > position));
  }

  public void reset(){
    m_elevatorMotorLeader.setPosition(0);
    m_elevatorMotorFollower.setPosition(0);
  }


  @Override
  public void periodic() {
    
  }

  @Override
  public void simulationPeriodic() {

  }

  public void setElevatorPID(DoublePreference P, DoublePreference D, DoublePreference I, DoublePreference G){
    m_Slot0Configs = new Slot0Configs();

    m_elevatorMotorLeader.getConfigurator().refresh(m_Slot0Configs);

    m_Slot0Configs.withKP(P.getValue()).withKD(D.getValue()).withKI(I.getValue()).withKG(G.getValue());

    m_elevatorMotorLeader.getConfigurator().apply(m_Slot0Configs);
    m_elevatorMotorFollower.getConfigurator().apply(m_Slot0Configs);
  }

  @Logged(name = "Actual kP", importance = Importance.CRITICAL)
  public double getElevatorkP(){
    m_Slot0Configs = new Slot0Configs();
    m_elevatorMotorLeader.getConfigurator().refresh(m_Slot0Configs);
    return m_Slot0Configs.kP;
  }

  @Logged(name = "Actual kD", importance = Importance.CRITICAL)
  public double getElevatorkD(){
    m_Slot0Configs = new Slot0Configs();
    m_elevatorMotorLeader.getConfigurator().refresh(m_Slot0Configs);
    return m_Slot0Configs.kD;
  }

  @Logged(name = "Actual kI", importance = Importance.CRITICAL)
  public double getElevatorkI(){
    m_Slot0Configs = new Slot0Configs();
    m_elevatorMotorLeader.getConfigurator().refresh(m_Slot0Configs);
    return m_Slot0Configs.kI;
  }

  @Logged(name = "Actual kG", importance = Importance.CRITICAL)
  public double getElevatorkG(){
    m_Slot0Configs = new Slot0Configs();
    m_elevatorMotorLeader.getConfigurator().refresh(m_Slot0Configs);
    return m_Slot0Configs.kG;
  }

  @Logged(name = "Voltage", importance = Importance.CRITICAL)
  public double getVoltage(){
    return m_elevatorMotorLeader.getMotorVoltage().getValueAsDouble();
  }

  @Logged(name = "Current", importance = Importance.CRITICAL)
  public double getCurrent(){
    return m_elevatorMotorLeader.getStatorCurrent().getValueAsDouble();
  }


}
