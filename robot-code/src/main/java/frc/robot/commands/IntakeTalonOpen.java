// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.commands;
import frc.robot.subsystems.GrabberTalon;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
public class IntakeTalonOpen extends CommandBase {
  /** Creates a new IntakeTalon. */
  private GrabberTalon m_talon;
  private double m_speed;
  public IntakeTalonOpen(GrabberTalon t) {
    m_talon = t;
    m_speed = 1.0; //open
    addRequirements(m_talon);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //m_talon.Intake(-1.0); //resets the talons to an open position
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_talon.Intake(m_speed);
  //runs periodically to ensure the talon doesnt overextend. Change value later
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //m_talon.Intake(-1.0); //closes if interrupted, idk if I want to include this or not
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(m_talon.getBoreEncoder() < Constants.boreThreshold) { //if the bore encoder is less than a certain value 
    return true;
    }
    return false; //return trues once the condition above is not true
  }
}