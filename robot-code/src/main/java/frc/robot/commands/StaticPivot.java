// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;
public class StaticPivot extends CommandBase {
  private Arm m_arm;
  /** Creates a new StaticPivot. */
  public StaticPivot(Arm a) {
    m_arm = a;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_arm.getPivotEncoder() > Constants.EncoderConstants.staticValue + 2) {
      m_arm.rotatePivot(-1);
    }
    else if (m_arm.getPivotEncoder() < Constants.EncoderConstants.staticValue - 2) {
      m_arm.rotatePivot(1);
    }
  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Constants.EncoderConstants.staticValue = 0;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
