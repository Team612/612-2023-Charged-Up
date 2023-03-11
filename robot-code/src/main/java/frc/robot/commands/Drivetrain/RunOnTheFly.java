// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drivetrain;

import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.PoseEstimator;
import frc.robot.subsystems.Vision;

public class RunOnTheFly extends CommandBase {
  private final Drivetrain driveSystem;
  private final Vision m_vision;
  private final PoseEstimator poseEstimatorSystem;
  private final boolean resetOdom;
  private final TrajectoryCreation m_traj;
  private final boolean isBlueAlliance;

  private CommandBase controllerCommand = Commands.none();

  /** Creates a new RunOnTheFly. */
  public RunOnTheFly(Drivetrain d, PoseEstimator p, boolean resetOdom, boolean isBlueAlliance, TrajectoryCreation traj, Vision v) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.driveSystem = d;
    this.poseEstimatorSystem = p;
    this.resetOdom = resetOdom;
    this.isBlueAlliance = isBlueAlliance;
    this.m_traj = traj;
    this.m_vision = v;

    addRequirements(this.driveSystem, this.m_vision);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    PathPlannerTrajectory path = m_traj.onthefly(poseEstimatorSystem, m_vision, isBlueAlliance);

    if(resetOdom){
      driveSystem.resetOdometry();
    }

    controllerCommand = Drivetrain.followTrajectory(driveSystem, poseEstimatorSystem, path);
    controllerCommand.initialize();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    controllerCommand.execute();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    controllerCommand.end(interrupted);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return controllerCommand.isFinished();
  }
}