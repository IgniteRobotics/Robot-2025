package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import java.util.function.Supplier;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Robot;
import frc.robot.generated.TunerConstants;

public class Elevator implements Subsystem {
    private final TalonFX m_elevatorMotor;

  private Slot0Configs m_Slot0Configs = new Slot0Configs();
  
  private SoftwareLimitSwitchConfigs m_softLimitConfig = new SoftwareLimitSwitchConfigs();

  private MotionMagicConfigs m_motionMagicConfigs = new MotionMagicConfigs();

  private MotorOutputConfigs m_motorConfig = new MotorOutputConfigs();

  private TalonFXConfiguration m_fxCfg = new TalonFXConfiguration();

  public MotionMagicVoltage m_MMPosition =   new MotionMagicVoltage(0);


  /** Creates a new Climber. */
  public Elevator() {
    m_elevatorMotor = TunerConstants.ElevatorConstants.ELEVATOR_MOTOR; 

    m_Slot0Configs = TunerConstants.ElevatorConstants.createSlot0Configs(); 
    m_elevatorMotor.getConfigurator().apply(m_Slot0Configs);

    m_softLimitConfig = TunerConstants.ElevatorConstants.createSoftLimitConigs(); 
    m_elevatorMotor.getConfigurator().apply(m_softLimitConfig);

    m_motionMagicConfigs = TunerConstants.ElevatorConstants.createMotionMagicConfigs();
    m_elevatorMotor.getConfigurator().apply(m_motionMagicConfigs);

    m_motorConfig = TunerConstants.ElevatorConstants.createMotorOutputConfigs();
    m_elevatorMotor.getConfigurator().apply(m_motorConfig);

    Mechanism2d mech = new Mechanism2d(3, 3);
    MechanismRoot2d root = mech.getRoot("elevator", 2, 0);

  }

  public void setPositionRevolutions(double position) {
    m_elevatorMotor.setControl(m_MMPosition.withPosition(position));
  }


  @Override
  public void periodic() {

  }

}
