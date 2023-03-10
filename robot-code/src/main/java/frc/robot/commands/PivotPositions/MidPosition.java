// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.PivotPositions;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.EncoderConstants;
import frc.robot.subsystems.Arm;

public class MidPosition extends CommandBase {
  /** Creates a new MidPosition. */
  private Arm m_arm;
  public MidPosition(Arm arm) {
    m_arm = arm;
    addRequirements(arm);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //start not moving
    
    //clockwise is postive
    //counter clockwise is negative
    //clockwise up 
    //counterclock is down
    
    m_arm.rotatePivot(0); 
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //Rotate
    m_arm.rotatePivot(0.5); 

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //stop
    m_arm.rotatePivot(0); 
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //stop at the angle
    if(m_arm.getPivotEncoder() >= EncoderConstants.MidPositionCube) return true;
    return false;
  }
}
