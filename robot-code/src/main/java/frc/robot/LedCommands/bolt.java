// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.LedCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.led;
public class bolt extends CommandBase {
  private int counter;
  private led m_led;
  /** Creates a new bolt. */
  public bolt(led l) {
    m_led = l;
    addRequirements(m_led);
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
    counter++;
    if (counter < 6) {
    m_led.setLedSpecific(counter,255,0,0);
    }
    else {
      if (counter == m_led.getLength()){
        counter = 0;
      }
      m_led.setLedSpecific(counter-6, 0, 0, 0);
      m_led.setLedSpecific(counter, 255, 0, 0);
    } //temp colors
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
