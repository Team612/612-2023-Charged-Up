// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drivetrain;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.PivotPositions.DefenseMode;
import frc.robot.commands.PivotPositions.MoveToPosition;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Telescope;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Boop extends SequentialCommandGroup {
  /** Creates a new Boop. */
  private final Telescope m_scope;
  private final Arm m_arm;

  public Boop(Telescope scope, Arm arm) {
    m_scope = scope;
    m_arm = arm;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new DefenseMode(m_scope,0.5),
      new MoveToPosition(m_arm, 0.6, 20),
      new DefenseMode(m_scope, 0.5),
      new MoveToPosition(m_arm, 0.6, 0)
    );
  }
}
