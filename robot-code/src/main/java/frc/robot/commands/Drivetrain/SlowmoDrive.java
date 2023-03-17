// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.controls.ControlMap;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

public class SlowmoDrive extends CommandBase {
  Drivetrain m_drivetrain;
  /** Creates a new SlowmoDrive. */
  public SlowmoDrive(Drivetrain d) {
    m_drivetrain = d;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Constants.DrivetrainConstants.slowmo == 0.50){ //if the command is pressed again
      Constants.DrivetrainConstants.slowmo = 1.0;
    }
    else {
      Constants.DrivetrainConstants.slowmo = 0.50; 
    } //yes, its a command that changes one value, deal with it
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
   
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  
}
}
