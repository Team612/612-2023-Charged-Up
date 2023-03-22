// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.PivotPositions;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.EncoderConstants;
import frc.robot.commands.ExtendRetract;
import frc.robot.subsystems.Telescope;

public class DefenseMode extends CommandBase {
  /** Creates a new DefenseMode. */
  private final Telescope m_scope;
  private final double m_speed;
  private int start_timer;
  private int counter;
  public DefenseMode(Telescope scope, double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_scope = scope;
    m_speed = speed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    counter = 0;
    start_timer = 0;
  }
  public boolean isSpike(){
    if(start_timer >= 10 && m_scope.getCurrent() >= EncoderConstants.tele_motor_current){
      counter++;
    }
    else{
      counter = 0;
    }
    if(counter >= 5) return true;
    return false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    isSpike();
    start_timer++;
    m_scope.moveTelescope(-m_speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    start_timer = 0;
    m_scope.moveTelescope(0);
    // System.out.println("defense mode ended**********");
    ExtendRetract.freeze = false;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished(){
    if(m_scope.getLimitSwitch()) return true; // || isSpike()) return true;
    return false;
  }
}
