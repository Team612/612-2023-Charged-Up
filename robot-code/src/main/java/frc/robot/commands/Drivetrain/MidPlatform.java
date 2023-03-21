// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drivetrain;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class MidPlatform extends CommandBase {
  /** Creates a new MidPlatform. */
  private final Drivetrain m_drivetrain;
  private double initialPos;
  public MidPlatform(Drivetrain drivetrain) {
    m_drivetrain = drivetrain;
    addRequirements(drivetrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    initialPos = m_drivetrain.getEncoderPosition();
    m_drivetrain.driveMecanum(0, 0, 0, 0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drivetrain.driveMecanum(0.5, 0.5, 0.5, 0.5);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.driveMecanum(0, 0, 0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(m_drivetrain.getEncoderPosition() - initialPos) >= 1.25;
  }
}
