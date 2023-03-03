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
 
  /** Creates a new Pivot. */
  public TelescopeDetract(Telescope scope) {
    m_scope = scope;
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
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_scope.moveTelescope(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished(){
    return false;
  }
}
