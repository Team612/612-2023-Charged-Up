// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.LedCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.led;

public class Yellow extends CommandBase {
  private led m_led;
  private int counter;
  /** Creates a new PurpleYellow. */
  public Yellow(led led) {
    m_led = led;
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_led);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    counter = 0;
    m_led.yellow();
  }
      
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("**************************works***************************");
    counter++;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_led.ChantillyTheme();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (counter == 100){
      return true;
    }
    return false;
  }
}
