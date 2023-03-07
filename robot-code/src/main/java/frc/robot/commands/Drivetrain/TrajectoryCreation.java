
package frc.robot.commands.Drivetrain;

import java.util.List;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
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

        

        System.out.println("*************************** END PRINT **********************************");

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

    public Trajectory tuneAngle = TrajectoryGenerator.generateTrajectory(
        new Pose2d(0, 0, new Rotation2d(0)),
        List.of(new Translation2d(.5,0)),
        new Pose2d(1,0, new Rotation2d(Units.degreesToRadians(120))), 
        config
    );

    public Trajectory return_alignTrajectory(PhotonCamera camera, Translation2d finalPose){
        PhotonPipelineResult result = camera.getLatestResult();
        if(result.hasTargets()){
            PhotonTrackedTarget bestTarget = result.getBestTarget();
            Double yaw = -bestTarget.getYaw();
            
            Transform3d transform3d = bestTarget.getBestCameraToTarget();
            double x = transform3d.getX();
            double y = transform3d.getY();
            
            
            return TrajectoryGenerator.generateTrajectory(
                new Pose2d(0,0, new Rotation2d(0)),
                List.of(new Translation2d((x - finalPose.getX())/2,(y - finalPose.getY()) / 2)),
                new Pose2d(x - finalPose.getX(),y - finalPose.getY(), new Rotation2d(Units.degreesToRadians(yaw))),
                config
            );
             
        }
        else{
            // System.out.println("doesn't work, Arjun sucks");
            return TrajectoryGenerator.generateTrajectory(new Pose2d(0, 0, new Rotation2d(0)), List.of(new Translation2d(-0.5,0)),new Pose2d(-1,0, new Rotation2d(0)), config_backwards);
        }
    }
}