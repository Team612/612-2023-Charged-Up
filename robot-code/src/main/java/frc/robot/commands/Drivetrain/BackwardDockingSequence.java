// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drivetrain;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drivetrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class BackwardDockingSequence extends SequentialCommandGroup {
  /** Creates a new BackwardDockingSequence. */
  private final Drivetrain m_drivetrain;
  public BackwardDockingSequence(Drivetrain drivetrain) {
    m_drivetrain = drivetrain;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new BackwardDriveUp(m_drivetrain),
      new BackwardMidPlatform(m_drivetrain),
      new BackwardAutoBalance(m_drivetrain)
    );
  }
}
