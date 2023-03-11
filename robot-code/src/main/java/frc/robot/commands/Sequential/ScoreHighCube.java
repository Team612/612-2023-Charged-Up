// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Sequential;
import frc.robot.commands.AlignToTag;
import frc.robot.commands.Release;
import frc.robot.commands.TelescopeExtend;
import frc.robot.commands.PivotPositions.HighPositionCube;
import frc.robot.subsystems.Arm;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Telescope;
import frc.robot.subsystems.Grabber;
// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoreHighCube extends SequentialCommandGroup {
  Arm m_arm = new Arm();
  Telescope m_scope = new Telescope();
  Grabber m_grabber = new Grabber();
  /** Creates a new ScoreHighCube. */
  public ScoreHighCube() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new AlignToTag(),
      new HighPositionCube(m_arm),
      new TelescopeExtend(m_scope),
      new Release(m_grabber)
    );
  }
}
