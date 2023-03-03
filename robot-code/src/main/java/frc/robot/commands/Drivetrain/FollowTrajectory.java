// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drivetrain;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.PoseEstimator;
import frc.robot.Constants;

public class FollowTrajectory extends CommandBase {

      private final Drivetrain driveSystem;
      private final PoseEstimator poseEstimatorSystem;
      private final String pathName;
      private final PathConstraints constraints;
      private final boolean resetOdom;

      private CommandBase controllerCommand = Commands.none();

      public FollowTrajectory(
                  Drivetrain d, PoseEstimator p, String pathName,
                  PathConstraints constraints, boolean resetOdom) {
            this.driveSystem = d;
            this.poseEstimatorSystem = p;
            this.pathName = pathName;
            this.constraints = constraints;
            this.resetOdom = resetOdom;
      }

      @Override
      public void initialize() {
            var path = PathPlanner.loadPath(pathName, constraints);
            if (path == null) {
                  end(false);
                  return;
            }
            PathPlannerTrajectory alliancePath = PathPlannerTrajectory.transformTrajectoryForAlliance(
                        path,
                        Constants.DrivetrainConstants.alliance);

            if (resetOdom)
                  poseEstimatorSystem.setCurrentPose(alliancePath.getInitialHolonomicPose());
            controllerCommand = Drivetrain.followTrajectory(driveSystem, poseEstimatorSystem, alliancePath);
            controllerCommand.initialize();
      }

      @Override
      public void execute() {
            controllerCommand.execute();
      }

      @Override
      public void end(boolean interrupted) {
            controllerCommand.end(interrupted);
      }

      @Override
      public boolean isFinished() {
            return controllerCommand.isFinished();
      }
}