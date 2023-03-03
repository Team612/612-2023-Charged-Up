// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drivetrain;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPMecanumControllerCommand;
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

            return mecanumControllerCommand.andThen(
                  new InstantCommand (() -> 
                  drivetrain.mecanumVolts(
                        new MecanumDriveMotorVoltages(0,0,0,0)), drivetrain));

      }  


      // Assuming this method is part of a drivetrain subsystem that provides the necessary methods
      public Command generatePathPlannerTrajectoryCommand(PathPlannerTrajectory traj, boolean isFirstPath, Drivetrain drivetrain, PoseEstimator estimator) {
            return new SequentialCommandGroup(
            new InstantCommand(() -> {
                  // Reset odometry for the first path you run during auto
                  // if(isFirstPath){
                  // estimator.setCurrentPose(traj.getInitialHolonomicPose());
                  // }
            }),

            new PPMecanumControllerCommand(traj, 
                  estimator::getCurrentPose, // Pose supplier
                  Constants.DrivetrainConstants.kDriveKinematics, // MecanumDriveKinematics
                  new PIDController(Constants.DrivetrainConstants.kPXController, 0, 0), // X controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
                  new PIDController(Constants.DrivetrainConstants.kPYController, 0, 0), // Y controller (usually the same values as X controller)
                  new PIDController(Constants.DrivetrainConstants.kPThetaController, 0, 0), // Rotation controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
                  2.5, // Max wheel velocity meters per second
                  drivetrain.getCurrentWheelSpeedsConsumer(),
                  drivetrain,
                  estimator
                  )
            );
      }
}