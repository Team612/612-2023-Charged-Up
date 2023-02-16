
package frc.robot.commands.Drivetrain;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
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
    
    public Trajectory return_Trajectory(PhotonCamera camera, Vision m_vision, Pose3d finalPose){
        if (camera.getLatestResult().hasTargets()){
            PhotonPipelineResult result = camera.getLatestResult();
            PhotonTrackedTarget bestTarget = result.getBestTarget();
            Pose3d initialPose3d = m_vision.return_camera_pose_tag(bestTarget.getFiducialId(), result);
            Pose2d initialPose = initialPose3d.toPose2d();
            
            return TrajectoryGenerator.generateTrajectory(
                new Pose2d(initialPose.getX(), initialPose.getY(), new Rotation2d(bestTarget.getYaw())), 
                List.of(new Translation2d( initialPose.getX() + ((finalPose.getX() - initialPose.getX())/2), initialPose.getY() + (finalPose.getY() - initialPose.getY())/2)),
                new Pose2d(finalPose.getX(), finalPose.getY(), new Rotation2d(0)),
                config
            );
        }
        else{ 
            System.out.println("doesn't work, Arjun sucks");
            return TrajectoryGenerator.generateTrajectory(new Pose2d(0, 0, new Rotation2d(0)), List.of(new Translation2d(-0.5,0)),new Pose2d(-1,0, new Rotation2d(0)), config);
        }
    }

    public Trajectory StrafeRightMeter(){   
        return TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, new Rotation2d(0)),
            List.of(new Translation2d(0,-0.5)),
            new Pose2d(0,-1-.62, new Rotation2d(Units.degreesToRadians(0))),
            config_backwards
        );
    }

    public Trajectory StrafeLeftMeter(){   
        return TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, new Rotation2d(0)),
            List.of(new Translation2d(0,0.5)),
            new Pose2d(0,1+.62, new Rotation2d(Units.degreesToRadians(0))),
            config
        );
    }

    public Trajectory ForwardMeter(){
        return TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, new Rotation2d(0)),
            List.of(new Translation2d(.5,0)),
            new Pose2d(1,0, new Rotation2d(Units.degreesToRadians(0))),
            config
        );
    }

    public Trajectory BackwardMeter(){
        return TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, new Rotation2d(0)),
            List.of(new Translation2d(-0.5,0)),
            new Pose2d(-1.1,0, new Rotation2d(Units.degreesToRadians(0))),
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