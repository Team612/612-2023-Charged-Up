
package frc.robot.commands.Drivetrain;

import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.subsystems.PoseEstimator;
import frc.robot.subsystems.Vision;

public class TrajectoryCreation {

    public TrajectoryConfig config = new TrajectoryConfig(Constants.DrivetrainConstants.kMaxVelocityMetersPerSecond, Constants.DrivetrainConstants.maxAccelerationMetersPerSecondSq)
        .setKinematics(Constants.DrivetrainConstants.kDriveKinematics);

    public TrajectoryConfig config_backwards = new TrajectoryConfig(Constants.DrivetrainConstants.kMaxVelocityMetersPerSecond, Constants.DrivetrainConstants.maxAccelerationMetersPerSecondSq)
        .setKinematics(Constants.DrivetrainConstants.kDriveKinematics).setReversed(true);
    

    public Trajectory testTrajectory = TrajectoryGenerator.generateTrajectory(
        new Pose2d(0, 0, new Rotation2d(0)),
        List.of(new Translation2d(2,0), new Translation2d(2,-2), new Translation2d(0, -2)),
        new Pose2d(0,0, new Rotation2d(0)), 
        config); 
    
    public Trajectory StrafeRightMeter(PoseEstimator estimation){ 
        Pose2d estimatedPose = estimation.getCurrentPose();
        double x = estimatedPose.getX();
        double y = estimatedPose.getY();
        double degrees = estimatedPose.getRotation().getRadians();
        
        return TrajectoryGenerator.generateTrajectory(
            new Pose2d(x, y, new Rotation2d(degrees)),
            List.of(new Translation2d(x,y-0.5)),
            new Pose2d(x,y-1, new Rotation2d(Units.degreesToRadians(degrees))),
            config_backwards
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

        //figure out a way to find an initial position 

        // System.out.println("*************************** PRINT **********************************");
        // var currentPose = estimation.getCurrentPose();
        // System.out.println(currentPose);
        Pose2d estimatedPose = estimation.getCurrentPose();
        double x = estimatedPose.getX();
        double y = estimatedPose.getY();
        double degrees = estimatedPose.getRotation().getRadians();

        

        // System.out.println("*************************** END PRINT **********************************");

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
            List.of(new Translation2d(x-2.5,y)),
            new Pose2d(x-4.75, y, new Rotation2d(degrees
            )),
            config_backwards
        );
    }

    public PathPlannerTrajectory onthefly(PoseEstimator estimation, Vision vision, boolean isBlueAlliance, double y_translation){
        Pose2d estimatedPose = estimation.getCurrentPose();
        System.out.println("**************" + estimatedPose + "********************");        
        double x = estimatedPose.getX();
        double y = estimatedPose.getY();
        Rotation2d angle = estimatedPose.getRotation();
        int id = vision.getCamera().getLatestResult().getBestTarget().getFiducialId();
        Pose2d tagPose = vision.return_tag_pose(id).toPose2d();
        double tagX = tagPose.getX();
        double tagY = tagPose.getY();
        System.out.println(angle);


        if(isBlueAlliance){
            // Rotation2d displacement = new Rotation2d(Units.degreesToRadians(-180)).minus(angle);
            return PathPlanner.generatePath(
                new PathConstraints(Constants.DrivetrainConstants.kMaxVelocityMetersPerSecond, 
                Constants.DrivetrainConstants.maxAccelerationMetersPerSecondSq),
                new PathPoint(new Translation2d(x, y), new Rotation2d(), angle), 
                new PathPoint(new Translation2d(tagX + 1.2, tagY+y_translation), new Rotation2d(), new Rotation2d(Units.degreesToRadians(-180)))
            );
        }
        else{
            return PathPlanner.generatePath(
                new PathConstraints(Constants.DrivetrainConstants.kMaxVelocityMetersPerSecond, 
                Constants.DrivetrainConstants.maxAccelerationMetersPerSecondSq),
                new PathPoint(new Translation2d(x, y), angle),
                new PathPoint(new Translation2d(tagX - 1, tagY-y_translation), angle)
            );
        }
    }
}