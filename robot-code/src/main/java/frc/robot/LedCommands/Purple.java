// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.LedCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.led;

public class Purple extends CommandBase {
  private led m_led;
  private RobotContainer m_container;
  private int counter;
  /** Creates a new PurpleYellow. */
  public Purple(led led) {
    m_led = led;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_led);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    counter = 0;
    m_led.purple();
  }
      
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    counter++;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (counter == 100){
      return true;
    }
    return false;
  }
}
