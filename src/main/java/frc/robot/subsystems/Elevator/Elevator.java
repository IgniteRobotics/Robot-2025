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
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Preferences.DoublePreference;

@Logged
public class Elevator implements Subsystem {
    private final TalonFX m_elevatorMotor;

  private Slot0Configs m_Slot0Configs = new Slot0Configs();
  
  private SoftwareLimitSwitchConfigs m_softLimitConfig = new SoftwareLimitSwitchConfigs();

  private MotionMagicConfigs m_motionMagicConfigs = new MotionMagicConfigs();

  private MotorOutputConfigs m_motorConfig = new MotorOutputConfigs();

  private TalonFXConfiguration m_fxCfg = new TalonFXConfiguration();

  public MotionMagicVoltage m_MMPosition =   new MotionMagicVoltage(0);

  //logged stuff
  @Logged(name = "Target Position", importance = Importance.CRITICAL)
  private double m_targetPosition;

  //Mech stuff
  Mechanism2d mech;
  MechanismRoot2d root;
  MechanismLigament2d arm;
  MechanismLigament2d wrist;


  /** Creates a new Climber. */
  public Elevator() {
    m_elevatorMotor = ElevatorConstants.ELEVATOR_MOTOR; 

    m_Slot0Configs = ElevatorConstants.createSlot0Configs(); 
    m_elevatorMotor.getConfigurator().apply(m_Slot0Configs);

    m_softLimitConfig = ElevatorConstants.createSoftLimitConigs(); 
    m_elevatorMotor.getConfigurator().apply(m_softLimitConfig);

    m_motionMagicConfigs = ElevatorConstants.createMotionMagicConfigs();
    m_elevatorMotor.getConfigurator().apply(m_motionMagicConfigs);

    m_motorConfig = ElevatorConstants.createMotorOutputConfigs();
    m_elevatorMotor.getConfigurator().apply(m_motorConfig);

    mechConfigure();

  }

  private void mechConfigure(){
    mech = new Mechanism2d(3, 3);
    root = mech.getRoot("elevator", 2, 0);
    arm = root.append(new MechanismLigament2d("arm", 1, 90));
    wrist =
        arm.append(
            new MechanismLigament2d("wrist", 0.5, 90, 6, new Color8Bit(Color.kPurple)));
    SmartDashboard.putData("Mech2d", mech);
  }

  @NotLogged
  public void setPositionRevolutions(double position) {
    m_targetPosition = position;
    m_elevatorMotor.setControl(m_MMPosition.withPosition(position).withSlot(0));
  }
  
  @NotLogged
  public void setPositionRevolutions(DoublePreference position){
    setPositionRevolutions(position.get());
  }

  public void alterMech(double length, double angle){
    arm.setLength(length);
    wrist.setAngle(angle);
    SmartDashboard.putData("Mech2d", mech);

  }


  @Override
  public void periodic() {
    
  }

  @Override
  public void simulationPeriodic() {

  }

  public void setElevatorPID(DoublePreference P, DoublePreference D, DoublePreference I, DoublePreference G){
    m_Slot0Configs = new Slot0Configs();

    m_elevatorMotor.getConfigurator().refresh(m_Slot0Configs);

    m_Slot0Configs.withKP(P.getValue()).withKD(D.getValue()).withKI(I.getValue()).withKG(G.getValue());

    m_elevatorMotor.getConfigurator().apply(m_Slot0Configs);
  }

  @Logged(name = "Actual kP", importance = Importance.CRITICAL)
  public double getElevatorkP(){
    m_Slot0Configs = new Slot0Configs();
    m_elevatorMotor.getConfigurator().refresh(m_Slot0Configs);
    return m_Slot0Configs.kP;
  }

  @Logged(name = "Actual kD", importance = Importance.CRITICAL)
  public double getElevatorkD(){
    m_Slot0Configs = new Slot0Configs();
    m_elevatorMotor.getConfigurator().refresh(m_Slot0Configs);
    return m_Slot0Configs.kD;
  }

  @Logged(name = "Actual kI", importance = Importance.CRITICAL)
  public double getElevatorkI(){
    m_Slot0Configs = new Slot0Configs();
    m_elevatorMotor.getConfigurator().refresh(m_Slot0Configs);
    return m_Slot0Configs.kI;
  }

  @Logged(name = "Actual kG", importance = Importance.CRITICAL)
  public double getElevatorkG(){
    m_Slot0Configs = new Slot0Configs();
    m_elevatorMotor.getConfigurator().refresh(m_Slot0Configs);
    return m_Slot0Configs.kG;
  }

  @Logged(name = "Voltage", importance = Importance.CRITICAL)
  public double getVoltage(){
    return m_elevatorMotor.getMotorVoltage().getValueAsDouble();
  }

  @Logged(name = "Current", importance = Importance.CRITICAL)
  public double getCurrent(){
    return m_elevatorMotor.getStatorCurrent().getValueAsDouble();
  }


}
