// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.EncoderConstants;
import frc.robot.Constants.MotorSpeeds;
import frc.robot.subsystems.Telescope;

public class TelescopeExtend extends CommandBase {
  /** Creates a new Telescope. */
  private final Telescope m_scope;
  private final int counter;
 
  /** Creates a new Pivot. */
  public TelescopeExtend(Telescope scope) {
    m_scope = scope;
    counter = 0;
    addRequirements(m_scope);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //if (!m_scope.isExtended())
      m_scope.moveTelescope(-MotorSpeeds.tele_arm_speed); //extend is negative speeds, tele_arm_speed is positive
      System.out.println("Rate: " + m_scope.getTeleEncoderRate());
      System.out.println("Current: " + m_scope.getVoltage());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_scope.moveTelescope(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    /*
     if (m_scope.getTeleEncoderRate() <= EncoderConstants.tele_extension_rate - EncoderConstants.tele_extension_rate_thresh && m_scope.getVoltage() >= EncoderConstants.tele_motor_voltage + tele_motor_voltage_thresh) {
      counter++;
      if (counter == 5) {
        return true;
      } 
    }
    else counter = 0;

    return false;
  }
     */
    
    return false;
  }
}
