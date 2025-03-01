package frc.robot.subsystems.EndEffector;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

public class EndEffectorConstants {
    public static final double WRIST_POSITION_ERROR = 0.1;

    public static final int kCoralMotorId = 2;
        public static final int kAlgaeMotorId = 3;
        public static final int kWristMotorId = 4;

        public static final int kEnterBeamBreakId = 8;
        public static final int kPrepBeamBreakId = 9;
        public static final int kAlgaeBeamBreakId = 16;
        public static final int kIntakeFunnelBeanmBreakId = 17;

        public static final double CORAL_kV = 0;
        public static final double CORAL_kS = 0;
        public static final double CORAL_kP = 0;
        public static final double CORAL_kI = 0;
        public static final double CORAL_kD = 0;

        public static final double ALGAE_kV = 0;
        public static final double ALGAE_kS = 0;
        public static final double ALGAE_kP = 0;
        public static final double ALGAE_kI = 0;
        public static final double ALGAE_kD = 0;

        public static final double WRIST_kV_0 = 0;
        public static final double WRIST_kS_0 = 0.3;
        public static final double WRIST_kP_0 = 40;
        public static final double WRIST_kI_0 = 0;
        public static final double WRIST_kD_0 = 0;
        public static final double WRIST_kG_0 = 0.14;

        public static final double WRIST_kV_1 = 0;
        public static final double WRIST_kS_1 = 0;
        public static final double WRIST_kP_1 = 0;
        public static final double WRIST_kI_1 = 0;
        public static final double WRIST_kD_1 = 0;
        public static final double WRIST_kG_1 = 0.2;

        public static final double INTAKE_CORAL_VOLTAGE = 1;
        public static final double OUTTAKE_CORAL_VOLTAGE = -1;

        public static final double INTAKE_CORAL_POWER = 0.25;
        public static final double OUTTAKE_CORAL_POWER = 0.25;

        public static final double INTAKE_ALGAE_POWER = 0.25;
        public static final double OUTTAKE_ALGAE_POWER = 0.25;
        public static final double ALGAE_HOLD_POWER = 0.03;

        public static final double STOW_POSITION_WITH_ALGAE = 0;
        public static final double STOW_POSITION_NO_ALGAE = 0;


        public static Slot0Configs createCoralMotorSlot0Configs(){
            Slot0Configs slot = new Slot0Configs();
            slot.kV = EndEffectorConstants.CORAL_kV;
            slot.kS = EndEffectorConstants.CORAL_kS;
            slot.kP = EndEffectorConstants.CORAL_kP;
            slot.kI = EndEffectorConstants.CORAL_kI;
            slot.kD = EndEffectorConstants.CORAL_kD;
            return slot;
        }

        public static final int kWristCANcoderId = 18;
        public static final CANcoder m_wristCANcoder = new CANcoder(kWristCANcoderId);


        //Cancoder
        public static CANcoder getConfiguredCANcoder(){
            CANcoderConfiguration config = new CANcoderConfiguration();
            config.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
            config.MagnetSensor.MagnetOffset = -0.147;
            m_wristCANcoder.getConfigurator().apply(config);
            m_wristCANcoder.getAbsolutePosition().setUpdateFrequency(100);
            return m_wristCANcoder;
        }

        public static MotorOutputConfigs createCoralMotorOutputConfigs(){
            MotorOutputConfigs configs = new MotorOutputConfigs();
            configs.withInverted(InvertedValue.Clockwise_Positive);
            configs.NeutralMode = NeutralModeValue.Brake;
            return configs;
        }

        public static MotorOutputConfigs createAlgaeMotorOutputConfigs(){
            MotorOutputConfigs configs = new MotorOutputConfigs();
            configs.withInverted(InvertedValue.Clockwise_Positive);
            configs.NeutralMode = NeutralModeValue.Brake;
            return configs;
        }
        


        public static Slot0Configs createAlgaeMotorSlot0Configs(){
            Slot0Configs slot = new Slot0Configs();
            slot.kV = EndEffectorConstants.ALGAE_kV;
            slot.kS = EndEffectorConstants.ALGAE_kS;
            slot.kP = EndEffectorConstants.ALGAE_kP;
            slot.kI = EndEffectorConstants.ALGAE_kI;
            slot.kD = EndEffectorConstants.ALGAE_kD;
            return slot;
        }

        public static final StaticFeedforwardSignValue wristFeedforward = StaticFeedforwardSignValue.UseClosedLoopSign;
        public static final GravityTypeValue wristGravityType = GravityTypeValue.Arm_Cosine;
        public static Slot0Configs createWristMotorSlot0Configs(){
            Slot0Configs slot = new Slot0Configs();
            slot.kV = EndEffectorConstants.WRIST_kV_0;
            slot.kS = EndEffectorConstants.WRIST_kS_0;
            slot.kP = EndEffectorConstants.WRIST_kP_0;
            slot.kI = EndEffectorConstants.WRIST_kI_0;
            slot.kD = EndEffectorConstants.WRIST_kD_0;
            slot.kG = EndEffectorConstants.WRIST_kG_0;
            slot.StaticFeedforwardSign = wristFeedforward;
            slot.GravityType = wristGravityType;
            return slot;
        }

        public static Slot1Configs createWristMotorSlot1Configs(){
            Slot1Configs slot = new Slot1Configs();
            slot.kV = EndEffectorConstants.WRIST_kV_1;
            slot.kS = EndEffectorConstants.WRIST_kS_1;
            slot.kP = EndEffectorConstants.WRIST_kP_1;
            slot.kI = EndEffectorConstants.WRIST_kI_1;
            slot.kD = EndEffectorConstants.WRIST_kD_1;
            slot.kG = EndEffectorConstants.WRIST_kG_1;
            slot.StaticFeedforwardSign = wristFeedforward;
            slot.GravityType = wristGravityType;
            return slot;
        }

        public static final double WRIST_FORWARD_SOFT_LIMIT = 100;
        public static final double WRIST_REVERSE_SOFT_LIMIT = 0;
        public static final int WRIST_TOLERANCE = 0;
        public static SoftwareLimitSwitchConfigs createWristSoftLimitConfigs(){
            SoftwareLimitSwitchConfigs configs = new SoftwareLimitSwitchConfigs();
            configs.ForwardSoftLimitEnable = false;
            configs.ReverseSoftLimitEnable = false;
            configs.ForwardSoftLimitThreshold = WRIST_FORWARD_SOFT_LIMIT;
            configs.ReverseSoftLimitThreshold = WRIST_REVERSE_SOFT_LIMIT;
            return configs;
        }
        
        public static TalonFXConfiguration createWristTalonFXConfigs(){
            TalonFXConfiguration configs = new TalonFXConfiguration();
            configs.Feedback.FeedbackRemoteSensorID = kWristCANcoderId;
            configs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
            configs.Feedback.RotorToSensorRatio = 15;
            configs.Feedback.SensorToMechanismRatio = 3.2;
            return configs;
        }

        public static MotorOutputConfigs createWristMotorOutputConfigs(){
            MotorOutputConfigs configs = new MotorOutputConfigs();
            configs.Inverted = InvertedValue.Clockwise_Positive;
            return configs;
        }



        public enum ALGAE{
            
            PROCESS(.15),
            REEF(0),
            BARGE(-0.15),
            STOW_FULL(-0.2),
            STOW_EMPTY(-0.25);

            public final double angle;
            ALGAE(double value){
                angle = value;
            }

        }



        
}
