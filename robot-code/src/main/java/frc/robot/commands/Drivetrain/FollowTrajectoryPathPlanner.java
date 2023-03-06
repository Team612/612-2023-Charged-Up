// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drivetrain;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.PoseEstimator;

public class FollowTrajectoryPathPlanner extends CommandBase {

  private final Drivetrain driveSystem;
  private final PoseEstimator poseEstimatorSystem;
  private final String pathName;
  private final PathConstraints constraints;
  private final boolean resetOdom;
  private final boolean isBlueAlliance;

  private CommandBase controllerCommand = Commands.none();

  /** Creates a new FollowTrajectoryPathPlanner. */
  public FollowTrajectoryPathPlanner(Drivetrain d, PoseEstimator p, String pathName, PathConstraints constraints, boolean resetOdom, boolean isBlueAlliance) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.driveSystem = d;
    this.poseEstimatorSystem = p;
    this.pathName = pathName;
    this.constraints = constraints;
    this.resetOdom = resetOdom;
    this.isBlueAlliance = isBlueAlliance;

    addRequirements(this.driveSystem);
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    PathPlannerTrajectory path = PathPlanner.loadPath(pathName, constraints);
    if(path == null){
      end(false);
      return;
    }

    Alliance alliance;

    if(isBlueAlliance){
      alliance = Alliance.Red;
    }
    else{
      alliance = Alliance.Blue;
    }

    
    PathPlannerTrajectory alliancePath = PathPlannerTrajectory.transformTrajectoryForAlliance(path, alliance);
    if(resetOdom){
      driveSystem.resetOdometry();
    }

    controllerCommand = Drivetrain.followTrajectory(driveSystem, poseEstimatorSystem, alliancePath);
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
