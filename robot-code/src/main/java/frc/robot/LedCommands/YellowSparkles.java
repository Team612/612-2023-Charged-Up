// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.LedCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.led;
public class YellowSparkles extends CommandBase {
  private led m_led;
  /** Creates a new YellowSparkles. */
  public YellowSparkles(led l) {
    m_led = l;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_led.SparklePhase1();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
