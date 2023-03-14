// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.LedCommands;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.led;
public class YellowSparkle extends CommandBase {
  private final led m_led;
  private int tick;
  /** Creates a new YellowSparkle. */
  public YellowSparkle(led l) {
    m_led = l;
    addRequirements(m_led);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    tick = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() { //red: 255, green: 255, blue:0
    tick++;
    if (tick == 1){
      m_led.SparklePhase1();
    }
    if (tick == 11){
      m_led.SparklePhase2();
    }
    if (tick == 22){
      tick = 0;
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
