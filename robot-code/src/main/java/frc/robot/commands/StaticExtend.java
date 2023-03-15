// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Telescope;

public class StaticExtend extends CommandBase {
  /** Creates a new StaticPivot. */
  private final Telescope m_telescope;
  private double last_telescope_position;
  public StaticExtend(Telescope telescope) {
    m_telescope = telescope;
    addRequirements(telescope);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    last_telescope_position = m_telescope.getTeleEncoder();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_telescope.getTeleEncoder() > last_telescope_position - 2){
      m_telescope.moveTelescope(-0.3);
    }
    if(m_telescope.getTeleEncoder() < last_telescope_position + 2){
      m_telescope.moveTelescope(0.3);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
