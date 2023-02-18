// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;

public class TelescopeDetract extends CommandBase {
  /** Creates a new Telescope. */
  private final Arm m_arm;
  private final double m_speed;
 
  /** Creates a new Pivot. */
  public TelescopeDetract(Arm Arm, double speed) {
    m_arm = Arm;
    m_speed = speed;
    addRequirements(Arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_arm.rotateTelescope(-m_speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (m_arm.getArmDistance() > Constants.boreThreshold) {
        return true;
    }
    return false;
  }
}