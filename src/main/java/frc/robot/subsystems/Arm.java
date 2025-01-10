package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import java.util.function.Supplier;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.*;
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

import edu.wpi.first.epilogue.Logged;
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

@Logged
public class Arm implements Subsystem {
  private final TalonFX m_armMotor;

  private Slot0Configs m_Slot0Configs = new Slot0Configs();

  private Slot1Configs m_Slot1Configs = new Slot1Configs();

  private Slot2Configs m_Slot2Configs = new Slot2Configs();
  
  private SoftwareLimitSwitchConfigs m_softLimitConfig = new SoftwareLimitSwitchConfigs();

  private MotionMagicConfigs m_motionMagicConfigs = new MotionMagicConfigs();

  private MotorOutputConfigs m_motorConfig = new MotorOutputConfigs();

  private TalonFXConfiguration m_fxCfg = new TalonFXConfiguration();

  public MotionMagicVoltage m_MMPosition =   new MotionMagicVoltage(0);



  //Mech stuff IF NEEDED
  Mechanism2d mech;
  MechanismRoot2d root;
  MechanismLigament2d arm;
  MechanismLigament2d wrist;

  public DoublePreference armLength = new DoublePreference("armLength", 1);
  public DoublePreference wristAngle = new DoublePreference("wristAngle", 90);


  public Arm() {
    m_armMotor = TunerConstants.ArmConstants.ARM_MOTOR; 

    m_Slot0Configs = TunerConstants.ArmConstants.createSlot0Configs(); 
    m_armMotor.getConfigurator().apply(m_Slot0Configs);

    m_Slot1Configs = TunerConstants.ArmConstants.createSlot1Configs();
    m_armMotor.getConfigurator().apply(m_Slot1Configs);

    m_Slot2Configs = TunerConstants.ArmConstants.createSlot2Configs();
    m_armMotor.getConfigurator().apply(m_Slot2Configs);

    m_softLimitConfig = TunerConstants.ArmConstants.createSoftLimitConigs(); 
    m_armMotor.getConfigurator().apply(m_softLimitConfig);

    m_motionMagicConfigs = TunerConstants.ArmConstants.createMotionMagicConfigs();
    m_armMotor.getConfigurator().apply(m_motionMagicConfigs);

    m_motorConfig = TunerConstants.ArmConstants.createMotorOutputConfigs();
    m_armMotor.getConfigurator().apply(m_motorConfig);

  }

  /* 
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
  */

  public void setPosition(double angle){
    m_armMotor.setPosition(angle);
  }


  @Override
  public void periodic() {
  }

  @Override
  public void simulationPeriodic() {
  }

}
