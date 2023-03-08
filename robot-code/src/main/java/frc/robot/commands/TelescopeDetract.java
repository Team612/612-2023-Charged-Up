// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.EncoderConstants;
import frc.robot.Constants.MotorSpeeds;
import frc.robot.controls.ControlMap;
import frc.robot.subsystems.Telescope;

public class TelescopeDetract extends CommandBase {
  /** Creates a new Telescope. */
  private final Telescope m_scope;
  private int counter;
  private int start_timer;
 
  /** Creates a new Pivot. */
  public TelescopeDetract(Telescope scope) {
    m_scope = scope;
    counter = 0;
    start_timer = 0;
    addRequirements(m_scope);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute(){
    //if (!m_scope.isRetracted())
      m_scope.moveTelescope(MotorSpeeds.tele_arm_speed); //detract is positive speeds, tele_arm_speed is positive
      start_timer++;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_scope.moveTelescope(0);
    start_timer = 0;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished(){
    if(start_timer >= 10 && m_scope.getCurrent() >= (EncoderConstants.tele_motor_voltage + EncoderConstants.tele_motor_voltage_thresh)){
      if (m_scope.getTeleEncoderRate() <= EncoderConstants.tele_extension_rate - EncoderConstants.tele_extension_rate_thresh && m_scope.getCurrent() >= EncoderConstants.tele_motor_voltage + EncoderConstants.tele_motor_voltage_thresh) {
        counter++;
        if (counter == 5) {
          return true;
        } 
      }
      else counter = 0;
  
   }
   return false;
  }
}
