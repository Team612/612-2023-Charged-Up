// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Telescope;
public class TelescopeExtend extends CommandBase {
  /** Creates a new TelescopeExtend. */
  private Telescope m_scope;
  private final double m_speed;
  public TelescopeExtend(Telescope scope, double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_scope = scope;
    m_speed = speed;
    addRequirements(m_scope);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_scope.rotateTelescope(m_speed);
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
