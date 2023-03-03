// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drivetrain;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPMecanumControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.MecanumDriveMotorVoltages;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.PoseEstimator;

/** Add your docs here. */
public class FollowTrajectoryPathPlanner {
    Drivetrain m_drivetrain;
    public Command generateTrajectory(Drivetrain drivetrain, PathPlannerTrajectory traj, PoseEstimator estimator){
        PPMecanumControllerCommand mecanumControllerCommand = 
            new PPMecanumControllerCommand(
                traj, 
                estimator::getCurrentPose,
                Constants.DrivetrainConstants.kDriveKinematics,
                new PIDController(Constants.DrivetrainConstants.kPXController, 0, 0),
                new PIDController(Constants.DrivetrainConstants.kPYController, 0, 0),
                new PIDController(Constants.DrivetrainConstants.kPThetaController, 0, 0),
                Constants.DrivetrainConstants.kMaxVelocityMetersPerSecond,
                this.m_drivetrain.getCurrentWheelSpeedsConsumer(),
                true,
                drivetrain);

        return(new InstantCommand(() -> drivetrain.resetOdometry()).andThen(Commands.print("wef"))
        //run ppmecanumcontroller
        .andThen(mecanumControllerCommand)
        //make sure robot stops
        .andThen(new InstantCommand (() -> drivetrain.mecanumVolts(new MecanumDriveMotorVoltages(0,0,0,0)), drivetrain)));
    }
}
