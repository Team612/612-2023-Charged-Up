// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.controls.ControlMap;
import frc.robot.subsystems.Arm;

public class Pivot extends CommandBase {
  private final Arm m_arm;
 
  /** Creates a new Pivot. */
  public Pivot(Arm arm) {
    m_arm = arm;
    addRequirements();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_arm.rotatePivot1(ControlMap.gunner.getRawAxis(0) * 0.3);
    //m_arm.rotatePivot2(ControlMap.gunner.getRawAxis(0) * -0.2);
  }

  // Called once the command ends or is interrupted.
  @Override                                                                                                                                                                                                                                                                                                                                                                                                                                                                   public void end(boolean interrupted) {
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;//m_arm.ifBorePassesLimit();
  }
}
