
package frc.robot.commands.Drivetrain;

import java.util.List;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.subsystems.PoseEstimator;

public class TrajectoryCreation {


    public TrajectoryConfig config = new TrajectoryConfig(Constants.DrivetrainConstants.kMaxVelocityMetersPerSecond, Constants.DrivetrainConstants.maxAccelerationMetersPerSecondSq)
        .setKinematics(Constants.DrivetrainConstants.kDriveKinematics);

    public TrajectoryConfig config_backwards = new TrajectoryConfig(Constants.DrivetrainConstants.kMaxVelocityMetersPerSecond, Constants.DrivetrainConstants.maxAccelerationMetersPerSecondSq)
        .setKinematics(Constants.DrivetrainConstants.kDriveKinematics).setReversed(true);
    
    
    public Trajectory StrafeRightMeter(PoseEstimator estimation){ 
        Pose2d estimatedPose = estimation.getCurrentPose();
        double x = estimatedPose.getX();
        double y = estimatedPose.getY();
        double degrees = estimatedPose.getRotation().getRadians();
        
        return TrajectoryGenerator.generateTrajectory(
            new Pose2d(x, y, new Rotation2d(Units.degreesToRadians(degrees))),
            List.of(new Translation2d(x,y-0.5)),
            new Pose2d(x,y-1, new Rotation2d(Units.degreesToRadians(degrees))),
            config
        );
    }

    public Trajectory StrafeLeftMeter(PoseEstimator estimation){ 
        Pose2d estimatedPose = estimation.getCurrentPose();
        double x = estimatedPose.getX();
        double y = estimatedPose.getY();
        double degrees = estimatedPose.getRotation().getRadians();
        
        return TrajectoryGenerator.generateTrajectory(
            new Pose2d(x, y, new Rotation2d(degrees)),
            List.of(new Translation2d(x,y+0.5)),
            new Pose2d(x,y+1, new Rotation2d(Units.degreesToRadians(degrees))),
            config
        );
    }

    public Trajectory ForwardMeter(PoseEstimator estimation){
        Pose2d estimatedPose = estimation.getCurrentPose();
        double x = estimatedPose.getX();
        double y = estimatedPose.getY();
        double degrees = estimatedPose.getRotation().getRadians();
        return TrajectoryGenerator.generateTrajectory(
            new Pose2d(x, y, new Rotation2d(degrees)),
            List.of(new Translation2d(x + 0.5,y)),
            new Pose2d(x + 1.0, y, new Rotation2d(degrees)),
            config
        );
    }

    public Trajectory BackwardMeter(PoseEstimator estimation){
        Pose2d estimatedPose = estimation.getCurrentPose();
        double x = estimatedPose.getX();
        double y = estimatedPose.getY();
        double degrees = estimatedPose.getRotation().getRadians();

        return TrajectoryGenerator.generateTrajectory(
            new Pose2d(x, y, new Rotation2d(degrees)),
            List.of(new Translation2d(x-0.5,y)),
            new Pose2d(x-1.0, y, new Rotation2d(degrees
            )),
            config_backwards
        );
    }


    public Trajectory square(PoseEstimator estimation){
        Pose2d estimatedPose = estimation.getCurrentPose();
        double x = estimatedPose.getX();
        double y = estimatedPose.getY();
        double degrees = estimatedPose.getRotation().getRadians();

        return TrajectoryGenerator.generateTrajectory(
            new Pose2d(x, y, new Rotation2d(degrees)),
            List.of(new Translation2d(x+1,y), new Translation2d(x+1, y+1), new Translation2d(x, y+1)),
            new Pose2d(x, y, new Rotation2d(degrees)),
            config_backwards
        );
    }



    public Trajectory LeaveAndDock(PoseEstimator estimation){
        Pose2d estimatedPose = estimation.getCurrentPose();
        double x = estimatedPose.getX();
        double y = estimatedPose.getY();
        double degrees = estimatedPose.getRotation().getRadians();

        return TrajectoryGenerator.generateTrajectory(
            new Pose2d(x, y, new Rotation2d(degrees)),
            List.of(
                new Translation2d(x, y-1.5),
                new Translation2d(x-2.5,y-1.5), 
                new Translation2d(x-4, y-1.5), 
                new Translation2d(x-4, y-0.5)
            ),
            new Pose2d(x-2.5, y, new Rotation2d(degrees)),
            config_backwards
        );
    }
}