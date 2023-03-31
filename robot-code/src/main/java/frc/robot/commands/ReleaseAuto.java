// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Grabber;
public class ReleaseAuto extends CommandBase {
  /** Creates a new releaseTeleop. */
  private Grabber m_grabber;
  private int counter;
  public ReleaseAuto(Grabber g) {
    m_grabber = g;
    addRequirements(g);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    counter = 0;
    m_grabber.grab(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_grabber.grab(0.2);
    counter++;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_grabber.grab(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(counter >= 10) {
      return true;
    }
    return false;
  }
}
