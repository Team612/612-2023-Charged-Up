// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.PivotPositions;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;

public class MoveToPosition extends CommandBase {
  /** Creates a new HighPositionCone. */
  private Arm m_arm;
  private double m_speed;
  private double m_pivot_position;
  private TrapezoidProfile m_profile;
  private final double m_pivot_threshold;

  public MoveToPosition(Arm arm, double speed, double pivot_position) {
    m_arm = arm;
    m_speed = speed;
    m_pivot_position = pivot_position;
    m_pivot_threshold = 2;
    addRequirements(arm);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // if (m_arm.getPivotEncoder() >= m_pivot_position) {
    //   m_arm.rotatePivot(-m_speed);
    // } else if (m_arm.getPivotEncoder() <= m_pivot_position) {
    //   m_arm.rotatePivot(m_speed);
    // }
    m_arm.rotatePID(m_pivot_position);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_arm.rotatePivot(0);
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(m_arm.getPivotEncoder() >= m_pivot_position - m_pivot_threshold &&  m_arm.getPivotEncoder() <= m_pivot_position + m_pivot_threshold) return true;
    return false;
  }
}
