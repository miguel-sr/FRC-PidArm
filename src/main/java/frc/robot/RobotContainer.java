// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.ArmSubsystem;

public class RobotContainer {
  
  private final ArmSubsystem m_robotArm = new ArmSubsystem();

  Joystick m_armController = new Joystick(OIConstants.kArmControllerPort);

  public RobotContainer() {
    configureBindings();
  }
  
  private void configureBindings() {
    new JoystickButton(m_armController, OIConstants.kAButton)
      .onTrue(Commands.runOnce(
        () -> {
          m_robotArm.setGoal(2);
          m_robotArm.enable();
        }, 
        m_robotArm));

    // Move the arm to neutral position when the 'B' button is pressed.
    new JoystickButton(m_armController, OIConstants.kBButton)
      .onTrue(Commands.runOnce(
        () -> {
          m_robotArm.setGoal(ArmConstants.kArmOffsetRads);
          m_robotArm.enable();
        }, 
        m_robotArm));
  }

  public void disablePIDSubsystems() {
    m_robotArm.disable();
  }
  
  public Command getAutonomousCommand() {
    return null;
  }
}
