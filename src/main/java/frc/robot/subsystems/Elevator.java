package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.DynamicMotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Preferences.DoublePreference;
import frc.robot.generated.TunerConstants;

@Logged
public class Elevator extends SubsystemBase {

  private final TalonFX m_elevatorMotorLeader;

  private final TalonFX m_elevatorMotorFollower;

  private Slot0Configs m_Slot0Configs = new Slot0Configs();
  
  private SoftwareLimitSwitchConfigs m_softLimitConfig = new SoftwareLimitSwitchConfigs();

  private MotionMagicConfigs m_motionMagicConfigs = new MotionMagicConfigs();

  private MotorOutputConfigs m_motorConfig = new MotorOutputConfigs();

  public DynamicMotionMagicTorqueCurrentFOC m_MMPosition; 
  
  public SysIdRoutine m_RevolSysIdRoutine;

  //random thing
  private final VoltageOut m_voltReq = new VoltageOut(0.0);

  public Elevator() {
    m_elevatorMotorLeader = TunerConstants.ElevatorConstants.Elevator_MOTOR_LEADER; 
    m_elevatorMotorFollower = TunerConstants.ElevatorConstants.Elevator_MOTOR_FOLLOWER;

    m_Slot0Configs = TunerConstants.ElevatorConstants.createSlot0Configs(); 
    m_elevatorMotorLeader.getConfigurator().apply(m_Slot0Configs);
    m_elevatorMotorFollower.getConfigurator().apply(m_Slot0Configs);

    m_softLimitConfig = TunerConstants.ElevatorConstants.createSoftLimitConigs(); 
    m_elevatorMotorFollower.getConfigurator().apply(m_softLimitConfig);
    m_elevatorMotorLeader.getConfigurator().apply(m_softLimitConfig);

    m_motionMagicConfigs = TunerConstants.ElevatorConstants.createMotionMagicConfigs();
    m_elevatorMotorLeader.getConfigurator().apply(m_motionMagicConfigs);
    m_elevatorMotorFollower.getConfigurator().apply(m_motionMagicConfigs);

    m_motorConfig = TunerConstants.ElevatorConstants.createMotorOutputConfigs();
    m_elevatorMotorLeader.getConfigurator().apply(m_motorConfig);
    m_elevatorMotorFollower.getConfigurator().apply(m_motorConfig);

    m_RevolSysIdRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(
        null,
        Volts.of(4),
        null,
        (state) -> SignalLogger.writeString("state", state.toString())
      ), 
      new SysIdRoutine.Mechanism(
        (volts) -> this.setControl(m_voltReq.withOutput(volts.in(Volts))),
        null, 
        this)
    );

    m_MMPosition = new DynamicMotionMagicTorqueCurrentFOC(0, TunerConstants.ElevatorConstants.Elevator_VELOCITY,
      TunerConstants.ElevatorConstants.Elevator_ACCELERATION, TunerConstants.ElevatorConstants.Elevator_JERK);
  } 

  public void setControl(VoltageOut request){
    m_elevatorMotorLeader.setControl(request);
    m_elevatorMotorFollower.setControl(request);
  }
  public void setPositionRevolutions(double position){
    m_elevatorMotorLeader.setControl(m_MMPosition.withPosition(position));
    m_elevatorMotorFollower.setControl(m_MMPosition.withPosition(position));
  }

  public void setVoltage(double volts){
    m_elevatorMotorLeader.setVoltage(volts);
    m_elevatorMotorFollower.setVoltage(volts);
  }

  @Override
  public void periodic() {
  }

  @Override
  public void simulationPeriodic() {
  }
  
  public Command RevolSysIdTestBuilder(double staticTimeout, double dynamicTimeout){ 
    return m_RevolSysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward).withTimeout(staticTimeout)
      .andThen(m_RevolSysIdRoutine.quasistatic(SysIdRoutine.Direction.kReverse).withTimeout(staticTimeout))
      .andThen(m_RevolSysIdRoutine.dynamic(SysIdRoutine.Direction.kForward).withTimeout(dynamicTimeout))
      .andThen(m_RevolSysIdRoutine   .dynamic(SysIdRoutine.Direction.kReverse).withTimeout(dynamicTimeout))
      .finallyDo(() -> this.setVoltage(0));
  }

}