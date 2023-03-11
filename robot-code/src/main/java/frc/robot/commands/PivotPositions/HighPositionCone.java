// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.PivotPositions;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;

public class HighPositionCone extends CommandBase {
  /** Creates a new HighPositionCone. */
  private Arm m_arm;
  public HighPositionCone(Arm arm) {
    m_arm = arm;
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
    if (m_arm.getPivotEncoder() >= Constants.EncoderConstants.HighPositionCone) {
      m_arm.rotatePivot(-0.5);
    } else if (m_arm.getPivotEncoder() <= Constants.EncoderConstants.HighPositionCone) {
      m_arm.rotatePivot(0.5);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_arm.rotatePivot(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(m_arm.getPivotEncoder() == Constants.EncoderConstants.HighPositionCone) return true;
    return false;
  }
}
