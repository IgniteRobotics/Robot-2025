package frc.robot.subsystems.EndEffector;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class EndEffectorConstants {
    public static final int kCoralMotorId = 2;
        public static final int kAlgaeMotorId = 3;
        public static final int kWristMotorId = 4;

        public static final int kEnterBeamBreakId = 8;
        public static final int kPrepBeamBreakId = 9;

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

        public static final double WRIST_kV = 0;
        public static final double WRIST_kS = 0;
        public static final double WRIST_kP = 0;
        public static final double WRIST_kI = 0;
        public static final double WRIST_kD = 0;
        public static final double WRIST_kG = 0;

        public static final double INTAKE_CORAL_VOLTAGE = 1;
        public static final double OUTTAKE_CORAL_VOLTAGE = -1;

        public static final double INTAKE_CORAL_POWER = 0.25;
        public static final double OUTTAKE_CORAL_POWER = 0.5;

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

        public static MotorOutputConfigs createCoralMotorOutputConfigs(){
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

        public static Slot0Configs createWirstMotorSlot0Configs(){
            Slot0Configs slot = new Slot0Configs();
            slot.kV = EndEffectorConstants.WRIST_kV;
            slot.kS = EndEffectorConstants.WRIST_kS;
            slot.kP = EndEffectorConstants.WRIST_kP;
            slot.kI = EndEffectorConstants.WRIST_kI;
            slot.kD = EndEffectorConstants.WRIST_kD;
            slot.kG = EndEffectorConstants.WRIST_kG;
            return slot;
        }

        public static final double WRIST_FORWARD_SOFT_LIMIT = 100;
        public static final double WRIST_REVERSE_SOFT_LIMIT = 0;
        public static SoftwareLimitSwitchConfigs createWristSoftLimitConfigs(){
            SoftwareLimitSwitchConfigs configs = new SoftwareLimitSwitchConfigs();
            configs.ForwardSoftLimitEnable = false;
            configs.ReverseSoftLimitEnable = false;
            configs.ForwardSoftLimitThreshold = WRIST_FORWARD_SOFT_LIMIT;
            configs.ReverseSoftLimitThreshold = WRIST_REVERSE_SOFT_LIMIT;
            return configs;
        }



        public enum ALGAE{
            
            PROCESS(1),
            BARGE(2);

            public final double height;
            ALGAE(double value){
                height = value;
            }

        }



        
}
