// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.LedCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.led;
public class YellowSparkleRandom extends CommandBase {
  private led m_led;
  private int tick1;
  private int tick2;
  private int tick3;
  private int tick4;
  /** Creates a new YellowSparkleRandom. */
  public YellowSparkleRandom(led l) {
    m_led = l;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    for (int i = 0; i < m_led.getLength(); i++) {
      m_led.flashRandom(i,100,100,100);
      m_led.flashRandom(i,100,100,100);
  }
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
