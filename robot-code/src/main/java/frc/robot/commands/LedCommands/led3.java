// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.LedCommands;
import frc.robot.subsystems.led;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class led3 extends CommandBase {
  private led m_led;
  private int t = 0;
  /** Creates a new led3. */
  public led3(led l) {
    m_led = l;
    addRequirements(m_led);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_led.led3(t);
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (t++ == 120) {
      return true;
    }
    return false;
  }
}
