// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.controls.ControlMap;
import frc.robot.subsystems.Drivetrain;

public class DefaultDrive extends CommandBase {
  /** Creates a new DefaultDrive. */
  Drivetrain m_drivetrain;
  public DefaultDrive(Drivetrain drivetrain) {
    m_drivetrain = drivetrain;
    addRequirements(drivetrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drivetrain.driveMecanum(0, 0, 0, 0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drivetrain.FieldOrientedDrive(-ControlMap.driver.getRawAxis(1), ControlMap.driver.getRawAxis(0), ControlMap.driver.getRawAxis(4));
    // m_drivetrain.driveMecanum(-ControlMap.driver.getRawAxis(1), ControlMap.driver.getRawAxis(0), ControlMap.driver.getRawAxis(4));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.driveMecanum(0, 0, 0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
