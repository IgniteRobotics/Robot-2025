// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.subsystems.Arm;
import frc.robot.Preferences.DoublePreference;

public class RobotContainer {

    private final static CommandXboxController driverController = new CommandXboxController(0);    

    public final Arm arm = new Arm();

    /* Path follower */
    private final SendableChooser<Command> autoChooser = new SendableChooser<>();

    public DoublePreference motorAngle = new DoublePreference("armAngle", 90);

    //Mech stuff
    Mechanism2d mech = new Mechanism2d(3, 3);
    MechanismRoot2d root = mech.getRoot("endEffector", 2, 0);
    MechanismLigament2d m_elevatorAvatar;
    MechanismLigament2d m_armAvatar;
    
    public RobotContainer() {
        autoChooser.addOption("Test Arm", arm.turnSysIdTestBuilder(2, 2));
        SmartDashboard.putData("Auto Mode", autoChooser);
        configureMech();
        configureBindings();
    }

    private void configureBindings() {
        driverController.x().onTrue(new InstantCommand(() -> arm.setPosition(motorAngle.getValue())).andThen(
        new InstantCommand(() -> m_armAvatar.setAngle(motorAngle.getValue()))).andThen(    
        new InstantCommand(() -> SmartDashboard.putData("Mech2d", mech))));
    }

    private void configureMech(){
        m_elevatorAvatar = root.append(new MechanismLigament2d("elevator", 2, 90));
        m_armAvatar = m_elevatorAvatar.append(new MechanismLigament2d("arm", 0.5, 90, 6, 
            new Color8Bit(Color.kPurple)));
        SmartDashboard.putData("Mech2d", mech);
    }

    public Command getAutonomousCommand() {
        /* First put the drivetrain into auto run mode, then run the auto */
        return autoChooser.getSelected();
    }
}
