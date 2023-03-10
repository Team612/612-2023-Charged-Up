// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.led;

public class Purple extends CommandBase {

  Drivetrain m_drivetrain;
  led m_led;
  /** Creates a new PurpleYellow. */
  public Purple(Drivetrain drivetrain, led led) {
    m_drivetrain = drivetrain;
    m_led = led;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_drivetrain, m_led);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_led.purple();
  }
      
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
