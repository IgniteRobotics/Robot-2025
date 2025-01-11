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
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Robot;
import frc.robot.Preferences.DoublePreference;
import frc.robot.generated.TunerConstants;

public class Elevator implements Subsystem {
    private final TalonFX m_elevatorMotor;

  private Slot0Configs m_Slot0Configs = new Slot0Configs();
  
  private SoftwareLimitSwitchConfigs m_softLimitConfig = new SoftwareLimitSwitchConfigs();

  private MotionMagicConfigs m_motionMagicConfigs = new MotionMagicConfigs();

  private MotorOutputConfigs m_motorConfig = new MotorOutputConfigs();

  private TalonFXConfiguration m_fxCfg = new TalonFXConfiguration();

  public MotionMagicVoltage m_MMPosition =   new MotionMagicVoltage(0);

  //Mech stuff
  Mechanism2d mech;
  MechanismRoot2d root;
  MechanismLigament2d arm;
  MechanismLigament2d wrist;


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

  public void setPositionRevolutions(double position) {
    m_elevatorMotor.setControl(m_MMPosition.withPosition(position));
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

}
