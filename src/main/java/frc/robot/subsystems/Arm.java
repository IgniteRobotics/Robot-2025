package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.DynamicMotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Preferences.DoublePreference;
import frc.robot.generated.TunerConstants;

@Logged
public class Arm extends SubsystemBase {
  private final TalonFX m_armMotor;

  private final CANcoder m_CANcoder;

  private TalonFXConfiguration m_TalonFXConfiguration = new TalonFXConfiguration();

  private CANcoderConfiguration m_CANcoderConfiguration = new CANcoderConfiguration();

  private Slot0Configs slot0Configs = new Slot0Configs();
  
  private SoftwareLimitSwitchConfigs m_softLimitConfig = new SoftwareLimitSwitchConfigs();

  private MotionMagicConfigs m_motionMagicConfigs = new MotionMagicConfigs();

  private MotorOutputConfigs m_motorConfig = new MotorOutputConfigs();

  public MotionMagicVoltage m_MMPosition; 

  public PositionVoltage m_PVPosition;
  
  public SysIdRoutine m_RotSysIdRoutine;

  //random thing
  private final VoltageOut m_voltReq = new VoltageOut(0.0);

  public Arm() {
    m_armMotor = TunerConstants.ArmConstants.ARM_MOTOR_Leader; 

    m_CANcoder = TunerConstants.ArmConstants.kArmCANcoder;

    m_CANcoderConfiguration = new CANcoderConfiguration();
    m_CANcoder.getConfigurator().refresh(m_CANcoderConfiguration);
    TunerConstants.ArmConstants.createCANcoderConfiguration(m_CANcoderConfiguration);
    m_CANcoder.getConfigurator().apply(m_CANcoderConfiguration);

    m_TalonFXConfiguration = TunerConstants.ArmConstants.createTalonFXConfiguration();
    m_armMotor.getConfigurator().apply(m_TalonFXConfiguration);

    m_softLimitConfig = TunerConstants.ArmConstants.createSoftLimitConigs(); 
    m_armMotor.getConfigurator().apply(m_softLimitConfig);

    m_motionMagicConfigs = TunerConstants.ArmConstants.createMotionMagicConfigs();
    m_armMotor.getConfigurator().apply(m_motionMagicConfigs);

    m_motorConfig = TunerConstants.ArmConstants.createMotorOutputConfigs();
    m_armMotor.getConfigurator().apply(m_motorConfig);

    slot0Configs = TunerConstants.ArmConstants.createSlot0Configs();
    m_armMotor.getConfigurator().apply(slot0Configs);

    m_RotSysIdRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(
        null,
        Volts.of(2),
        null,
        (state) -> SignalLogger.writeString("state", state.toString())
      ), 
      new SysIdRoutine.Mechanism(
        (volts) -> m_armMotor.setControl(m_voltReq.withOutput(volts.in(Volts))),
        null, 
        this)
    );

    m_MMPosition = new MotionMagicVoltage(0);
    //m_armMotor.setControl(m_MMPosition.withSlot(0));
  } 

  public void setPositionDegrees(double angle){
    this.setPositionRotations(angle/360.0);
  }

  public void setPositionRotations(double rotations){
   // m_armMotor.setControl(m_MMPosition.withSlot(0).withPosition(rotations));
   m_PVPosition = new PositionVoltage(rotations).withSlot(0);
   m_armMotor.setControl(m_PVPosition);
  }

  public double getPosition(){
    return m_armMotor.getPosition().getValueAsDouble();
  }

  public void setVoltage(double volts){
    m_armMotor.setVoltage(volts);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Arm Position From Motor", getPosition());
    SmartDashboard.putNumber("Arm Position from CANcoder", m_CANcoder.getPosition().getValueAsDouble());
  }

  @Override
  public void simulationPeriodic() {
  }

  public boolean compareForwardEndpoint(){
    return getPosition() >= TunerConstants.ArmConstants.SYSID_ARM_FORWARD_SOFT_LIMIT;
  }

  public boolean compareReverseEndpoint(){
    return getPosition() <= TunerConstants.ArmConstants.SYSID_ARM_REVERSE_SOFT_LIMIT;
  }
  
  public Command rotSysIdTestBuilder(){ 
    return m_RotSysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward).until(this::compareForwardEndpoint)
      .andThen(m_RotSysIdRoutine.quasistatic(SysIdRoutine.Direction.kReverse).until(this::compareReverseEndpoint))
      .andThen(m_RotSysIdRoutine.dynamic(SysIdRoutine.Direction.kForward).until(this::compareForwardEndpoint))
      .andThen(m_RotSysIdRoutine.dynamic(SysIdRoutine.Direction.kReverse).until(this::compareReverseEndpoint))
      .finallyDo(() -> this.setVoltage(0));
  }

  //temporary method
  public void setArmPID(DoublePreference P, DoublePreference D, DoublePreference I, DoublePreference S){
    slot0Configs = new Slot0Configs();

    m_armMotor.getConfigurator().refresh(slot0Configs);

    slot0Configs.withKP(P.getValue()).withKD(D.getValue()).withKI(I.getValue())
      .withKS(S.getValue());

    m_armMotor.getConfigurator().apply(slot0Configs);
  }

}
