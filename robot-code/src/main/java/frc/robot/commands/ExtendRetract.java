// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.ShuffleBoardButtons;
import frc.robot.Constants.MotorSpeeds;
import frc.robot.controls.ControlMap;
import frc.robot.subsystems.Telescope;

public class ExtendRetract extends CommandBase {
  /** Creates a new Telescope. */
  private final Telescope m_scope;
  private int counter;
  private int start_timer;
 
  /** Creates a new Pivot. */
  public ExtendRetract(Telescope scope) {
    m_scope = scope;
    addRequirements(m_scope);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    counter = 0;
    start_timer = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute(){
    if(ControlMap.GUNNER_LB.getAsBoolean()){
      m_scope.moveTelescope(MotorSpeeds.tele_arm_speed);
    }
    else if(ControlMap.GUNNER_RB.getAsBoolean()){
      m_scope.moveTelescope(-MotorSpeeds.tele_arm_speed);
      start_timer++;
      if (start_timer >= 5 && m_scope.getCurrent() >= ShuffleBoardButtons.teleSpikeThresh.getDouble(0)) { // m_scope.getTeleEncoderRate() >= ShuffleBoardButtons.teleEncoderRateThresh.getDouble(0) || 
        counter++;
      }
      else counter = 0;
    }
    else{
      m_scope.moveTelescope(0);
    }
    //m_scope.moveTelescope(-MotorSpeeds.tele_arm_speed); //detract is positive speeds, tele_arm_speed is positive
      
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
    if (counter >= 3) {
      m_scope.resetEncoder();
      System.out.println("tele encoder reset*******************");
      return true;
    }
   return false;
  }
}
