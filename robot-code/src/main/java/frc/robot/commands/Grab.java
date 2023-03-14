// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.ShuffleBoardButtons;
import frc.robot.Constants.MotorSpeeds;
import frc.robot.subsystems.Grabber;

public class Grab extends CommandBase {
  private final Grabber m_grabber;
  private int counter;
 
  /** Creates a new Pivot. */
  public Grab(Grabber grabber) {
    m_grabber = grabber;
    addRequirements(m_grabber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    counter = 0;
    m_grabber.setBooleanGrabber(true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    m_grabber.grab(MotorSpeeds.grabber_speed); //sticky grabber is positive for grab.
    System.out.println("*******GRABBING*********");

    if(m_grabber.getCurrent() >= ShuffleBoardButtons.grabberSpikeTresh.getDouble(0)){
      counter++;
    }
    else{
      counter = 0;
    }
    
    // if(start_timer >= 10 && m_grabber.getCurrent() >= EncoderConstants.grabber_motor_voltage){
    //   count++;
    // }
    // else count = 0;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_grabber.grab(0);
    m_grabber.setBooleanGrabber(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(counter >= 5) return true;
    return false;
  }
}