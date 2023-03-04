// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drivetrain;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPMecanumControllerCommand;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.kinematics.MecanumDriveMotorVoltages;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.MecanumControllerCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.PoseEstimator;

public class FollowTrajectory {
    public Command generateTrajectory(Drivetrain drivetrain, Trajectory m_traj, PoseEstimator estimator){
         
            MecanumControllerCommand mecanumControllerCommand =
            new MecanumControllerCommand(
            m_traj,
            estimator::getCurrentPose,
            Constants.DrivetrainConstants.kFeedforward,
            Constants.DrivetrainConstants.kDriveKinematics,
    
            //Position controllers 
            new PIDController(Constants.DrivetrainConstants.kPXController, 0, 0),
            new PIDController(Constants.DrivetrainConstants.kPYController, 0, 0),
            new ProfiledPIDController(Constants.DrivetrainConstants.kPThetaController, 0, 0, Constants.DrivetrainConstants.kThetaControllerConstraints),
    
            Constants.DrivetrainConstants.kMaxVelocityMetersPerSecond,

            //Velocity PID's
            new PIDController(Constants.DrivetrainConstants.kPFrontLeftVel, 0, 0),
            new PIDController(Constants.DrivetrainConstants.kPRearLeftVel, 0, 0),
            new PIDController(Constants.DrivetrainConstants.kPFrontRightVel, 0, 0),
            new PIDController(Constants.DrivetrainConstants.kPRearRightVel, 0, 0),
            drivetrain::getCurrentWheelSpeeds,
            drivetrain::mecanumVolts,
            estimator);


            MecanumControllerCommand positionMecanumControler = new MecanumControllerCommand(
                  m_traj, 
                  estimator::getCurrentPose,
                  Constants.DrivetrainConstants.kDriveKinematics,
                  new PIDController(Constants.DrivetrainConstants.kPXController, 0, 0),
                  new PIDController(Constants.DrivetrainConstants.kPYController, 0, 0),
                  new ProfiledPIDController(Constants.DrivetrainConstants.kPThetaController, 0, 0, Constants.DrivetrainConstants.kThetaControllerConstraints),
                  1,
                  drivetrain.getCurrentWheelSpeedsConsumer(),
                  drivetrain,
                  estimator
            );

            MecanumControllerCommandModified mecanumControllerCommandModified = new MecanumControllerCommandModified(
            m_traj,
            estimator::getCurrentPose,
            Constants.DrivetrainConstants.kFeedforward,
            Constants.DrivetrainConstants.kDriveKinematics,
    
            //Position controllers 
            new PIDController(Constants.DrivetrainConstants.kPXController, 0, 0),
            new PIDController(Constants.DrivetrainConstants.kPYController, 0, 0),
            new ProfiledPIDController(Constants.DrivetrainConstants.kPThetaController, 0, 0, Constants.DrivetrainConstants.kThetaControllerConstraints),
    
            Constants.DrivetrainConstants.kMaxVelocityMetersPerSecond,
            //Velocity PID's
            new PIDController(Constants.DrivetrainConstants.kPFrontLeftVel, 0, 0),
            new PIDController(Constants.DrivetrainConstants.kPRearLeftVel, 0, 0),
            new PIDController(Constants.DrivetrainConstants.kPFrontRightVel, 0, 0),
            new PIDController(Constants.DrivetrainConstants.kPRearRightVel, 0, 0),
            drivetrain::getCurrentWheelSpeeds,
            drivetrain::mecanumVolts,
            estimator);



            

            //setting up sequence of commands
            //resetting the drivetrain odometry

            return positionMecanumControler.andThen(
                  new InstantCommand (() -> 
                  drivetrain.mecanumVolts(
                        new MecanumDriveMotorVoltages(0,0,0,0)), drivetrain));

      } 
      
      



}