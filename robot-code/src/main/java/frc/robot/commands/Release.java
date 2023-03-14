// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.EncoderConstants;
import frc.robot.Constants.MotorSpeeds;
import frc.robot.controls.ControlMap;
import frc.robot.subsystems.Grabber;

public class Release extends CommandBase {
  private final Grabber m_grabber;
 
  /** Creates a new Pivot. */
  public Release(Grabber grabber) {
    m_grabber = grabber;
    //start_timer = 0;
    addRequirements(m_grabber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_grabber.grab(2*(ControlMap.gunner_joystick.getRawAxis(3)-0.5) * MotorSpeeds.grabber_speed * -1); //sticky grabber is negative for release
    
    System.out.println("*******RELEASING*********");
    // if(m_grabber.getCurrent() >= EncoderConstants.sticky_grabber_thresh){
    //   count++;
    // }
    // else count = 0;
    //start_timer++;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_grabber.grab(0);
    //start_timer = 0;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // if(count >= 5) return true;

    return false;
  }
}