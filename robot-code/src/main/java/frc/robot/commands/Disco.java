// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.led;
import frc.robot.subsystems.Drivetrain;
public class Disco extends CommandBase {
  private led m_led;
  private Drivetrain m_drivetrain;
  private int time;
  /** Creates a new Disco. */
  public Disco(led l) {
    m_led = l;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
      time = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if ((time % 1000) == 250){
      m_led.setLed(255,0,0);
    }
    else if((time % 1000) == 500){
      m_led.setLed(230,230,250);
    }
    else if ((time % 1000) == 750){
      m_led.setLed(0,255,0);
    }
    else if ((time % 1000) == 0){
      m_led.setLed(250,250,0);
    }
    time++; //seizure mode enabled
    m_drivetrain.driveMecanum(0.5,0.5,0,0);
  
    

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_led.resetled();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false; //disable it to turn off
  }
}
