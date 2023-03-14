// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.PivotPositions;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Telescope;

public class ExtendToPosition extends CommandBase {
  /** Creates a new ExtendToPosition. */
  private Telescope m_Telescope;
  private double m_speed;
  private double m_telescope_position;
  private final double m_threshold;



  public ExtendToPosition(Telescope telescope, double speed, double telescope_position) {
    m_Telescope = telescope;
    m_speed = speed;
    m_telescope_position = telescope_position;
    m_threshold = 2;
    addRequirements(telescope);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_Telescope.getTeleEncoder() >= m_telescope_position) {
      m_Telescope.moveTelescope(-m_speed);
    } else if (m_Telescope.getTeleEncoder() <= m_telescope_position) {
      m_Telescope.moveTelescope(m_speed);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Telescope.moveTelescope(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(m_Telescope.getTeleEncoder() >= m_telescope_position - m_threshold &&  m_Telescope.getTeleEncoder() <= m_telescope_position + m_threshold) return true;
    return false;
  }
}
