// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.LedCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.led;

public class Purple extends CommandBase {
  /** Creates a new Yellow. */
  private final led m_led;
  private int counter;
  public Purple(led led) {
    m_led = led;
    addRequirements(led);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    counter = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_led.purple();
    counter++;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return counter == 100;
  }
}
