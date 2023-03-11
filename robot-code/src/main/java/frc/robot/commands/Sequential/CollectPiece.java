// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Sequential;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Release;
import frc.robot.commands.TelescopeExtend;
import frc.robot.commands.Drivetrain.RunOnTheFly;
import frc.robot.commands.Drivetrain.TrajectoryCreation;
import frc.robot.commands.PivotPositions.LowPosition;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Grabber;
import frc.robot.subsystems.PoseEstimator;
import frc.robot.subsystems.Telescope;
import frc.robot.subsystems.Vision;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class CollectPiece extends SequentialCommandGroup {
  /** Creates a new CollectPiece. */
  public CollectPiece() {
      Drivetrain m_drivetrain = new Drivetrain();
      TrajectoryCreation m_traj = new TrajectoryCreation();
      PoseEstimator estimator = PoseEstimator.getPoseEstimatorInstance();
      Vision m_vision = Vision.getVisionInstance();
      Arm m_arm = new Arm();
      Telescope m_scope = new Telescope();
      Grabber m_grabber = new Grabber();
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new RunOnTheFly(m_drivetrain, estimator, true, true, m_traj, m_vision),
      new LowPosition(m_arm),
      new Release(m_grabber),
      new TelescopeExtend(m_scope)
    );
  }
}
