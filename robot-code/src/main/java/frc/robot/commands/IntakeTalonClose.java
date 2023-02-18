// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import frc.robot.subsystems.GrabberTalon;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class IntakeTalonClose extends CommandBase {
  /** Creates a new IntakeTalonClose. */
  private GrabberTalon m_talon;
  private double m_speed;
  public IntakeTalonClose(GrabberTalon t) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_speed = -1.0;
    m_talon =  t;
    addRequirements(m_talon);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_talon.Intake(m_speed);
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